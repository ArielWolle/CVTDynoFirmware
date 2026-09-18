#include "RpmCounter.h"
#include <pico/util/queue.h>

namespace {
// Ring buffer of recent edge timestamps per channel. Sized generously above any realistic
// edgesPerUpdate so a smoothing window can be widened at runtime without a rebuild.
constexpr size_t EDGE_BUF_SIZE = 64;

// A stale reading (no edges for this long) reports 0 RPM instead of holding a frozen value.
constexpr uint64_t RPM_STALE_TIMEOUT_US = 500000;
// Once stale, re-affirm the 0 reading at this cadence rather than every single loop1() iteration
// (which runs continuously with no delay) -- the value can't change while stale, so there's no
// benefit to re-timestamping it faster than this.
constexpr uint64_t RPM_STALE_REPORT_INTERVAL_US = 50000;

struct RpmChannelState {
  // Written only from the pin ISR; read from update() under noInterrupts() for atomicity (ISR and
  // update() both run on core1, so disabling interrupts is sufficient -- no cross-core access).
  uint64_t edgeBuf[EDGE_BUF_SIZE] = {0};
  volatile uint32_t bufHead = 0;         // next write index into edgeBuf
  volatile uint32_t totalEdges = 0;      // lifetime edge count, for staleness/diagnostics
  volatile uint64_t lastEdgeUs = 0;      // 0 == no edge ever seen
  volatile uint32_t interruptEvents = 0; // cleared by readAndClearInterruptEvents
  volatile uint32_t diagPulses = 0;      // cleared by readWindowCounts

  // spokes/edgesPerUpdate are written from core0 (serial command handling in main.cpp) but read
  // from core1 (RPM computation below) -- volatile for cross-core visibility, plain aligned
  // 16-bit reads/writes so no additional locking is needed for these two.
  volatile uint16_t spokes = 1;
  volatile uint16_t edgesPerUpdate = 1;

  // Core1-only bookkeeping (never touched from core0 or the ISR), safe without a lock.
  uint32_t lastReportedEdges = 0;
  uint64_t lastStaleReportUs = 0;
  bool wasStaleReported = false;
};

RpmChannelState primaryState;
RpmChannelState secondaryState;
bool interruptTestMode = false;

// Raw-edge transport is deliberately separate from the existing RPM estimator. The Pico SDK queue
// is IRQ- and multicore-safe: Core 1 produces events from the pin ISRs while Core 0 drains them for
// telemetry. A bounded queue plus per-channel overrun counters makes acquisition loss observable.
constexpr uint32_t RAW_EDGE_QUEUE_CAPACITY = 512;
queue_t rawEdgeQueue;
volatile bool rawEdgeQueueReady = false;
volatile bool rawPrimaryEnabled = true;
volatile bool rawSecondaryEnabled = true;
volatile uint32_t primaryRawOverruns = 0;
volatile uint32_t secondaryRawOverruns = 0;

void enqueueRawEdge(uint8_t channel, uint64_t timestampUs, uint32_t edgeIndex) {
  if (!rawEdgeQueueReady) return;
  if ((channel == 0 && !rawPrimaryEnabled) || (channel == 1 && !rawSecondaryEnabled)) return;

  const RpmCounter::RpmEdgeEvent event{timestampUs, edgeIndex, channel};
  if (!queue_try_add(&rawEdgeQueue, &event)) {
    if (channel == 0) {
      primaryRawOverruns++;
    } else {
      secondaryRawOverruns++;
    }
  }
}


void recordEdge(RpmChannelState& s, uint8_t channel) {
  const uint64_t nowUs = time_us_64();
  s.edgeBuf[s.bufHead % EDGE_BUF_SIZE] = nowUs;
  s.bufHead = s.bufHead + 1;
  s.totalEdges++;
  s.lastEdgeUs = nowUs;
  s.diagPulses++;
  if (interruptTestMode) s.interruptEvents++;
  enqueueRawEdge(channel, nowUs, s.totalEdges);
}

void primaryEdge() { recordEdge(primaryState, 0); }
void secondaryEdge() { recordEdge(secondaryState, 1); }

// Edge-triggered reciprocal counting: as soon as `edgesPerUpdate` new edges have landed since the
// last report, compute RPM from the exact elapsed time spanning them -- exact and unbiased
// regardless of RPM or acceleration, with no smoothing lag and no fixed polling window. With the
// default edgesPerUpdate=1 this recomputes on every single edge (e.g. ~900-1100 Hz for 12-16 tooth
// wheels at 4000-4600 RPM), far above the old fixed 20ms/50Hz window. Raising edgesPerUpdate trades
// update latency for immunity to tooth-spacing manufacturing tolerance, if that noise ever matters
// more than raw speed for a given wheel -- left to the app/operator to tune per RpmCounter.h.
void computeChannel(RpmChannelState& s, const uint64_t nowUs, uint32_t& rpmOut, uint64_t& captureOut, bool& readyOut) {
  readyOut = false;
  const uint16_t edgesPerUpdate = s.edgesPerUpdate; // snapshot once; only used to size the read below

  // Single lock window covering totalEdges/lastEdgeUs/bufHead and (when needed) the two edge-buffer
  // samples they index into, so nothing else can advance bufHead/overwrite edgeBuf between deciding
  // which slots to read and actually reading them.
  noInterrupts();
  const uint32_t totalEdges = s.totalEdges;
  const uint64_t lastEdgeUs = s.lastEdgeUs;
  const uint32_t head = s.bufHead;
  uint64_t newest = 0;
  uint64_t oldest = 0;
  const bool haveSpan = totalEdges > (uint32_t)edgesPerUpdate;
  if (haveSpan) {
    newest = s.edgeBuf[(head + EDGE_BUF_SIZE - 1) % EDGE_BUF_SIZE];
    oldest = s.edgeBuf[(head + EDGE_BUF_SIZE - 1 - edgesPerUpdate) % EDGE_BUF_SIZE];
  }
  interrupts();

  const bool stale = (lastEdgeUs == 0) || (nowUs - lastEdgeUs) > RPM_STALE_TIMEOUT_US;
  if (stale) {
    if (!s.wasStaleReported || (nowUs - s.lastStaleReportUs) >= RPM_STALE_REPORT_INTERVAL_US) {
      rpmOut = 0;
      captureOut = nowUs;
      readyOut = true;
      s.wasStaleReported = true;
      s.lastStaleReportUs = nowUs;
      s.lastReportedEdges = totalEdges;
    }
    return;
  }

  const uint32_t edgesSinceReport = totalEdges - s.lastReportedEdges;
  // Require totalEdges > edgesPerUpdate (not just >=) so "oldest" above always points at a real
  // edge the ISR has actually written, never an unwritten (zero) ring-buffer slot.
  if (edgesSinceReport >= edgesPerUpdate && haveSpan) {
    const uint64_t spanUs = newest - oldest;
    rpmOut = spanUs > 0
      ? (uint32_t)(60000000.0 * (double)edgesPerUpdate / ((double)spanUs * (double)s.spokes))
      : 0;
    captureOut = newest;
    readyOut = true;
    s.lastReportedEdges = totalEdges;
    s.wasStaleReported = false;
  }
}
}

namespace RpmCounter {
void begin(uint8_t primaryPin, uint8_t secondaryPin, uint16_t newPrimarySpokes, uint16_t newSecondarySpokes,
           uint16_t primaryEdgesPerUpdate, uint16_t secondaryEdgesPerUpdate) {
  primaryState = RpmChannelState();
  secondaryState = RpmChannelState();
  primaryState.spokes = newPrimarySpokes > 0 ? newPrimarySpokes : 1;
  secondaryState.spokes = newSecondarySpokes > 0 ? newSecondarySpokes : 1;
  primaryState.edgesPerUpdate = primaryEdgesPerUpdate > 0 ? primaryEdgesPerUpdate : 1;
  secondaryState.edgesPerUpdate = secondaryEdgesPerUpdate > 0 ? secondaryEdgesPerUpdate : 1;

  if (!rawEdgeQueueReady) {
    queue_init(&rawEdgeQueue, sizeof(RpmEdgeEvent), RAW_EDGE_QUEUE_CAPACITY);
    rawEdgeQueueReady = true;
  } else {
    RpmEdgeEvent staleEvent;
    while (queue_try_remove(&rawEdgeQueue, &staleEvent)) { }
  }
  rawPrimaryEnabled = true;
  rawSecondaryEnabled = true;
  primaryRawOverruns = 0;
  secondaryRawOverruns = 0;

  // Optoisolator outputs are normally open-collector/open-drain.
  pinMode(primaryPin, INPUT_PULLUP);
  pinMode(secondaryPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(primaryPin), primaryEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(secondaryPin), secondaryEdge, RISING);
}

void update(uint32_t& primaryRpm, uint32_t& secondaryRpm, uint64_t& primaryCaptureUs, uint64_t& secondaryCaptureUs,
            bool& primaryReady, bool& secondaryReady) {
  const uint64_t nowUs = time_us_64();
  computeChannel(primaryState, nowUs, primaryRpm, primaryCaptureUs, primaryReady);
  computeChannel(secondaryState, nowUs, secondaryRpm, secondaryCaptureUs, secondaryReady);
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

void setSpokes(uint8_t channel, uint16_t spokes) {
  if (channel == 0) {
    primaryState.spokes = spokes > 0 ? spokes : 1;
  } else if (channel == 1) {
    secondaryState.spokes = spokes > 0 ? spokes : 1;
  }
}

void getSpokes(uint16_t& primaryOut, uint16_t& secondaryOut) {
  primaryOut = primaryState.spokes;
  secondaryOut = secondaryState.spokes;
}

void setEdgesPerUpdate(uint8_t channel, uint16_t edgesPerUpdate) {
  const uint16_t clamped = edgesPerUpdate == 0 ? 1
    : (edgesPerUpdate > (EDGE_BUF_SIZE - 1) ? (uint16_t)(EDGE_BUF_SIZE - 1) : edgesPerUpdate);
  if (channel == 0) {
    primaryState.edgesPerUpdate = clamped;
  } else if (channel == 1) {
    secondaryState.edgesPerUpdate = clamped;
  }
}

void getEdgesPerUpdate(uint16_t& primaryOut, uint16_t& secondaryOut) {
  primaryOut = primaryState.edgesPerUpdate;
  secondaryOut = secondaryState.edgesPerUpdate;
}

void setInterruptTestMode(bool enabled) {
  interruptTestMode = enabled;
}

void setRawEdgeStreaming(uint8_t channel, bool enabled) {
  if (channel == 0) {
    rawPrimaryEnabled = enabled;
  } else if (channel == 1) {
    rawSecondaryEnabled = enabled;
  }
}

bool getRawEdgeStreaming(uint8_t channel) {
  if (channel == 0) return rawPrimaryEnabled;
  if (channel == 1) return rawSecondaryEnabled;
  return false;
}

bool tryReadRawEdge(RpmEdgeEvent& event) {
  if (!rawEdgeQueueReady) return false;
  return queue_try_remove(&rawEdgeQueue, &event);
}

void readRawEdgeDiagnostics(uint32_t& primaryOverruns, uint32_t& secondaryOverruns) {
  primaryOverruns = primaryRawOverruns;
  secondaryOverruns = secondaryRawOverruns;
}

}
