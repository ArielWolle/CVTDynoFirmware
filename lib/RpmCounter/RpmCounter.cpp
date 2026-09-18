#include "RpmCounter.h"

namespace {
// Per-channel SPSC (single-producer single-consumer) ring buffer of raw edge events. Producer is
// the pin ISR (primaryEdge()/secondaryEdge(), core1) or injectSyntheticEdge() (also core1, called
// from loop1()'s demo-mode branch -- guarded against the real ISR below since both run on core1).
// Consumer is popEdge(), called from core0. head/tail are only ever written by their respective
// single owner (producer owns head, consumer owns tail), so no lock is needed -- just memory
// barriers (__sync_synchronize()) to ensure each side's writes are visible to the other core in
// the right order before the index that "publishes" them is updated.
//
// Sized well above any realistic sustained edge rate (see the begin()/loop() cadence discussion in
// main.cpp) so overflow only happens if the consumer core genuinely stalls -- must be a power of
// two so index wrap can use a cheap mask instead of a modulo.
constexpr uint32_t EVENT_RING_SIZE = 256;
constexpr uint32_t EVENT_RING_MASK = EVENT_RING_SIZE - 1;

struct EdgeEvent {
  uint64_t edgeUs;
  uint32_t periodUs;
};

// A channel is considered stopped after this long without a real edge -- matches the old windowed
// implementation's staleness threshold. checkStale() (called every loop1() iteration, see
// RpmCounter::pollStale()) pushes one explicit periodUs=0 event when a channel crosses this, so
// the host actually sees RPM reach zero instead of the last nonzero reading being displayed
// forever once the wheel stops turning.
constexpr uint64_t RPM_STALE_TIMEOUT_US = 500000;

// A real tooth edge should never arrive faster than this after the previous one on the SAME
// channel -- guards against electrical noise/crosstalk on the input pin being misinterpreted as
// legitimate tooth data. Confirmed live on real hardware with no sensor connected: a sustained
// ~13,500 Hz noise burst on PIN_RPM1 (periods as low as 74us), coinciding with continuous
// ADS1256/SPI bus activity on the same board -- indistinguishable from real edges to the ISR
// without a check like this, and fast enough to flood the whole downstream pipeline (firmware TX
// bandwidth and the viewer's processing) with spurious telemetry. 200us (5,000 Hz / a period
// product of 300,000 RPM*teeth) is chosen comfortably below this while staying comfortably above
// demo mode's fastest legitimate period (~647us at its current bounds -- see
// injectDemoRpmEdges()'s demoRpm1/DEMO_RPM1_TEETH), so it can never reject real demo/sensor data
// for any physically reasonable primary/secondary pulley speed and tooth count.
//
// Note this reduces a sustained, dense noise burst by roughly two orders of magnitude rather than
// eliminating it outright: a rejected edge intentionally does NOT advance lastEdgeUs (see
// recordEdge() below), so once enough rejected edges accumulate that the elapsed time since the
// last ACCEPTED edge exceeds this threshold, the next one slips through as if it were a real,
// slower edge. If noise persists even with this filter in place, the real fix is hardware --
// shielding/routing on PIN_RPM1/PIN_RPM2, or an RC low-pass filter -- this is a software
// mitigation, not a substitute for that.
constexpr uint32_t MIN_VALID_PERIOD_US = 200;

struct RpmChannelState {
  EdgeEvent ring[EVENT_RING_SIZE] = {};
  volatile uint32_t head = 0;    // next slot the producer will write (producer-owned)
  volatile uint32_t tail = 0;    // next slot the consumer will read (consumer-owned)
  volatile uint32_t dropped = 0; // producer-owned; incremented instead of overwriting on overflow

  // Producer-only bookkeeping (only ever touched from core1 -- the ISR, injectSyntheticEdge(), and
  // checkStale(), all of which are guarded with noInterrupts()/interrupts() against each other
  // where they aren't already ISR-atomic -- so no additional lock is needed here).
  uint64_t lastEdgeUs = 0;   // 0 == no edge yet this "epoch" (since begin(), or since going stale)
  bool reportedStale = false; // true once the periodUs=0 "stopped" event has been pushed for this gap

  // Diagnostic counters, cleared independently by their own read-and-clear calls.
  volatile uint32_t totalEdges = 0;
  volatile uint32_t interruptEvents = 0;
  volatile uint32_t diagPulses = 0;
  volatile uint32_t rejectedNoise = 0; // edges rejected by MIN_VALID_PERIOD_US below
};

RpmChannelState primaryState;
RpmChannelState secondaryState;
bool interruptTestMode = false;

// Producer-side push -- called only from core1 (ISR or the interrupt-guarded synthetic injector).
// Drop-newest on overflow: if the consumer has fallen behind enough to fill the ring, we do NOT
// overwrite the oldest still-unread event (that would require the producer to also own/advance
// tail, breaking the single-owner-per-index invariant that makes this lock-free). Instead the new
// event is discarded and `dropped` incremented -- the consumer sees a gap (via the dropped
// counter) rather than corrupted/torn data.
void pushEdge(RpmChannelState& s, uint64_t edgeUs, uint32_t periodUs) {
  const uint32_t head = s.head;
  const uint32_t nextHead = (head + 1) & EVENT_RING_MASK;
  __sync_synchronize(); // fetch a fresh view of tail before deciding full/not-full
  if (nextHead == s.tail) {
    s.dropped++;
    return;
  }
  s.ring[head] = {edgeUs, periodUs};
  __sync_synchronize(); // event data must be visible before the consumer can see the new head
  s.head = nextHead;
}

// Only pushes a telemetry event when a period is actually computable (i.e. there was a previous
// edge to diff against) -- the very first edge since begin() or since a stale/idle reset just
// re-arms lastEdgeUs and otherwise does nothing, so periodUs is never fabricated as 0 for "no
// prior edge" the way the old design did. That frees periodUs=0 on the wire to mean exactly one
// thing: an explicit "stopped" report from checkStale() below, not an edge-case artifact.
void recordEdge(RpmChannelState& s) {
  const uint64_t nowUs = time_us_64();
  const bool havePrior = s.lastEdgeUs != 0;
  const uint32_t periodUs = havePrior ? (uint32_t)(nowUs - s.lastEdgeUs) : 0;
  // See MIN_VALID_PERIOD_US above -- deliberately does NOT touch lastEdgeUs/reportedStale/counters
  // on rejection, so the next genuinely-spaced edge is still timed against the last REAL edge (not
  // a noise pulse), and a burst of noise can never itself count as "the channel is active" for
  // staleness or diagnostic purposes.
  if (havePrior && periodUs < MIN_VALID_PERIOD_US) {
    s.rejectedNoise++;
    return;
  }
  s.lastEdgeUs = nowUs;
  s.reportedStale = false; // a real edge always clears any prior "stopped" report
  if (havePrior) pushEdge(s, nowUs, periodUs);
  s.totalEdges++;
  s.diagPulses++;
  if (interruptTestMode) s.interruptEvents++;
}

void primaryEdge() { recordEdge(primaryState); }
void secondaryEdge() { recordEdge(secondaryState); }

// Called every loop1() iteration (core1, same as the ISR/injector) for both channels. Pushes a
// single explicit periodUs=0 "stopped" event the first time a channel crosses RPM_STALE_TIMEOUT_US
// without a real edge, then resets lastEdgeUs to 0 so the *next* real edge is treated as a fresh
// "no prior edge" case (re-arming, not pushed) rather than computing a huge bogus period spanning
// the entire idle gap -- the edge after that one is the first real RPM reading post-restart.
void checkStale(RpmChannelState& s) {
  noInterrupts(); // guards against the real ISR preempting this, same as injectSyntheticEdge()
  const uint64_t nowUs = time_us_64();
  const uint64_t lastEdge = s.lastEdgeUs;
  const bool haveEdge = lastEdge != 0;
  if (haveEdge && !s.reportedStale && (nowUs - lastEdge) > RPM_STALE_TIMEOUT_US) {
    pushEdge(s, nowUs, 0);
    s.reportedStale = true;
    s.lastEdgeUs = 0;
  }
  interrupts();
}
}

namespace RpmCounter {
void begin(uint8_t primaryPin, uint8_t secondaryPin) {
  primaryState = RpmChannelState();
  secondaryState = RpmChannelState();

  // Optoisolator outputs are normally open-collector/open-drain.
  pinMode(primaryPin, INPUT_PULLUP);
  pinMode(secondaryPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(primaryPin), primaryEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(secondaryPin), secondaryEdge, RISING);
}

void pollStale() {
  checkStale(primaryState);
  checkStale(secondaryState);
}

bool popEdge(uint8_t channel, uint64_t& edgeUs, uint32_t& periodUs) {
  RpmChannelState& s = (channel == 0) ? primaryState : secondaryState;
  const uint32_t tail = s.tail;
  __sync_synchronize(); // fetch a fresh view of head before deciding empty/not-empty
  if (tail == s.head) {
    return false;
  }
  const EdgeEvent ev = s.ring[tail];
  __sync_synchronize(); // finish reading the slot before publishing that it's free again
  s.tail = (tail + 1) & EVENT_RING_MASK;
  edgeUs = ev.edgeUs;
  periodUs = ev.periodUs;
  return true;
}

void injectSyntheticEdge(uint8_t channel, uint32_t periodUs) {
  RpmChannelState& s = (channel == 0) ? primaryState : secondaryState;
  // Guards against the real pin ISR preempting this mid-push -- both run on core1, and unlike the
  // ISR (which can't preempt itself), this non-ISR call site genuinely can be interrupted by a
  // real edge arriving while demo mode is active (the pins/ISR stay attached regardless of
  // demo_mode; see main.cpp).
  noInterrupts();
  const uint64_t nowUs = time_us_64();
  pushEdge(s, nowUs, periodUs);
  s.lastEdgeUs = nowUs;
  s.totalEdges++;
  s.diagPulses++;
  interrupts();
}

uint32_t readAndClearDropped(uint8_t channel) {
  RpmChannelState& s = (channel == 0) ? primaryState : secondaryState;
  noInterrupts();
  const uint32_t d = s.dropped;
  s.dropped = 0;
  interrupts();
  return d;
}

uint32_t readAndClearRejectedNoise(uint8_t channel) {
  RpmChannelState& s = (channel == 0) ? primaryState : secondaryState;
  noInterrupts();
  const uint32_t n = s.rejectedNoise;
  s.rejectedNoise = 0;
  interrupts();
  return n;
}

void readDiagnostics(uint32_t& primaryEdges, uint32_t& secondaryEdges) {
  noInterrupts();
  primaryEdges = primaryState.totalEdges;
  secondaryEdges = secondaryState.totalEdges;
  interrupts();
}

void readWindowCounts(uint32_t& primaryCount, uint32_t& secondaryCount) {
  noInterrupts();
  primaryCount = primaryState.diagPulses;
  secondaryCount = secondaryState.diagPulses;
  primaryState.diagPulses = 0;
  secondaryState.diagPulses = 0;
  interrupts();
}

void readAndClearInterruptEvents(uint32_t& primaryEvents, uint32_t& secondaryEvents) {
  noInterrupts();
  primaryEvents = primaryState.interruptEvents;
  secondaryEvents = secondaryState.interruptEvents;
  primaryState.interruptEvents = 0;
  secondaryState.interruptEvents = 0;
  interrupts();
}

void setInterruptTestMode(bool enabled) {
  interruptTestMode = enabled;
}
}
