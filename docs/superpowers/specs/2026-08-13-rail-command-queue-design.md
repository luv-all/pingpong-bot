# Rail Command Queue — Design

## Problem

The linear rail (AXL-driven, `src/hardware/rail/axl_rail.rs`) cannot be commanded like a
Dynamixel joint. A Dynamixel accepts a new goal position at high frequency at any time. The
rail, once given a move command, must be allowed to finish (or be explicitly, safely stopped)
before a new command is sent — sending a new command while a previous one is still executing
today relies on `AxlLive::stop_if_moving` (`axl_live.rs:152`), which synchronously S-stops and
blocks the caller until idle before re-issuing. That is a low-level safety net inside a single
FFI call, not a systemic guarantee, and it doesn't compose well with a planned two-step
control flow: a fast **sparse** rail command sent as soon as a rough ball prediction is
available, followed later by an **exact** command once a refined prediction lands.

This design introduces a small, standalone, generic command queue that owns the rail driver on
a dedicated worker thread and guarantees: at most one command is "in flight," a newer command
always supersedes an older one that hasn't been sent yet, and the worker never sends a new
target until the previous one has genuinely finished.

This design covers only the queue module itself — not wiring it into `RealHardware` /
`control_worker.rs`. That integration is a follow-up once the queue is proven in isolation.

## Architecture

A new module, `src/hardware/rail/queue.rs`, exports a generic `RailQueue<R: RailDriver>`.

`RailQueue` is generic over a small trait rather than concrete over `AxlRail`, so its
concurrency behavior (latest-wins while a move is in flight) can be tested deterministically
with a mock driver, instead of relying on real AXL timing or `#[cfg(test)]` hooks inside
`AxlRail`:

```rust
pub trait RailDriver: Send {
    fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError>;
    fn wait_idle(&mut self) -> Result<(), HwError>;
}
```

`AxlRail` implements `RailDriver` by delegating to its existing `command_abs_in_secs` and
`wait_idle` methods — no changes to `axl_rail.rs`'s public API.

## Data structures & API

```rust
pub struct RailQueue<R: RailDriver> {
    shared: Arc<Shared>,
    handle: Option<JoinHandle<()>>,
    _driver: PhantomData<R>,
}

struct Shared {
    state: Mutex<QueueState>,
    cv: Condvar,
}

struct QueueState {
    pending: Option<PendingCommand>,
    moving: bool,
    last_error: Option<HwError>,
    shutdown: bool,
}

struct PendingCommand {
    target_m: f64,
    duration_secs: f64,
}

impl<R: RailDriver + 'static> RailQueue<R> {
    /// Spawns the worker thread, which takes ownership of `driver`.
    pub fn spawn(driver: R) -> Self;

    /// Overwrites any not-yet-sent pending command and wakes the worker.
    /// Never blocks. Does not return a Result — see Error handling.
    pub fn enqueue(&self, target_m: f64, duration_secs: f64);

    /// True if a command is currently executing on the rail, or one is
    /// queued waiting to be sent.
    pub fn is_moving(&self) -> bool;

    /// Takes (clears) the last recorded error, if any.
    pub fn take_error(&self) -> Option<HwError>;

    /// Blocks the caller until the queue is fully drained: no command
    /// executing and none pending.
    pub fn wait_idle(&self);
}
```

### Worker loop

```
loop {
    wait on condvar until state.pending.is_some() or state.shutdown
    if shutdown: return
    cmd = state.pending.take()
    state.moving = true; notify_all
    match driver.command_abs_in_secs(cmd.target_m, cmd.duration_secs) {
        Ok(_) => {
            if let Err(e) = driver.wait_idle() { state.last_error = Some(e) }
        }
        Err(e) => state.last_error = Some(e),
    }
    state.moving = false; notify_all
    // loop back — picks up whatever `pending` holds now, which may have
    // been overwritten one or more times while this command was executing
}
```

Because `enqueue()` can overwrite `pending` at any point — including while the worker is
mid-move — an "exact" command sent shortly after a "sparse" one always wins: the worker
finishes the sparse move (or observes the error), then immediately picks up the latest pending
target instead of the sparse one, with no backlog of stale intermediate commands ever sent to
the rail.

## Error handling

A failure in `command_abs_in_secs` or `wait_idle` is recorded in `last_error`, overwriting any
previous unread error, and the worker returns to waiting for the next command — **it keeps
processing future commands rather than halting**. A transient AXL error on one target
shouldn't permanently wedge the queue; the next `enqueue()` still gets a fair chance to run.
`take_error()` lets a caller poll and decide what to do with it (log, surface as a
`RuntimeEvent::Failed`, etc.) — this module does no logging or event dispatch itself.

Mutex locking uses `.lock().unwrap()`: the worker thread only ever calls `RailDriver` methods
that return `Result`, so a panic while holding the lock is not an expected path. Poison
recovery (as used elsewhere in `RealHardware`, which is reachable from multiple external
threads) is out of scope for this internal, single-worker module.

`Drop for RailQueue` sets `shutdown = true`, notifies the condvar, and joins the worker thread,
so tests and callers never leak the thread.

## Testing

All tests live in `#[cfg(test)] mod tests` in `queue.rs`, using a mock `RailDriver` that can be
told to block `wait_idle()` on a signal the test controls (e.g. a channel or `Barrier`), so
tests are deterministic rather than timing-based. No hardware or `windows`/`real` feature is
required.

- **Latest-wins under load**: enqueue target A with a driver whose `wait_idle()` blocks until
  released; while blocked, enqueue B then C; release; assert the driver only ever received
  `command_abs_in_secs` for A then C (B was overwritten and never sent).
- **`wait_idle()` blocks until fully drained**: the caller's `wait_idle()` returns only after
  both the in-flight move and any pending command have been sent and completed.
- **`is_moving()` reflects both executing and pending**: true while blocked on A; true again
  immediately after enqueueing C even though the worker hasn't picked it up yet; false once
  fully drained.
- **Error surfaces via `take_error()` without halting the queue**: mock driver returns `Err`
  for one target; assert `take_error()` returns it once (then `None` on a second call), and a
  subsequently enqueued target still gets sent and executed.
- **`Drop` joins the worker cleanly**: no panics or leaked threads when a `RailQueue` is
  dropped mid-move.

## Non-goals (this design)

- Wiring `RailQueue` into `RealHardware`/`control_worker.rs` — a separate follow-up.
- Strict FIFO execution of every enqueued command — explicitly rejected in favor of
  latest-wins, since stale intermediate ball-tracking targets are never useful once superseded.
- Halting the queue on error until explicitly cleared — explicitly rejected in favor of keeping
  the queue live for the next command.
