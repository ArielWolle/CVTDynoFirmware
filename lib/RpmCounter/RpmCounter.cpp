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

struct RpmChannelState {
  EdgeEvent ring[EVENT_RING_SIZE] = {};
  volatile uint32_t head = 0;    // next slot the producer will write (producer-owned)
  volatile uint32_t tail = 0;    // next slot the consumer will read (consumer-owned)
  volatile uint32_t dropped = 0; // producer-owned; incremented instead of overwriting on overflow

  // Producer-only bookkeeping (only ever touched from core1 -- the ISR and injectSyntheticEdge(),
  // which is itself guarded with noInterrupts()/interrupts() against the real ISR -- so no lock
  // needed here either).
  uint64_t lastEdgeUs = 0; // 0 == no edge yet this "epoch" (since begin(), or since diagnostics reset)

  // Diagnostic counters, cleared independently by their own read-and-clear calls.
  volatile uint32_t totalEdges = 0;
  volatile uint32_t interruptEvents = 0;
  volatile uint32_t diagPulses = 0;
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

void recordEdge(RpmChannelState& s) {
  const uint64_t nowUs = time_us_64();
  const uint32_t periodUs = (s.lastEdgeUs != 0) ? (uint32_t)(nowUs - s.lastEdgeUs) : 0;
  s.lastEdgeUs = nowUs;
  pushEdge(s, nowUs, periodUs);
  s.totalEdges++;
  s.diagPulses++;
  if (interruptTestMode) s.interruptEvents++;
}

void primaryEdge() { recordEdge(primaryState); }
void secondaryEdge() { recordEdge(secondaryState); }
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
