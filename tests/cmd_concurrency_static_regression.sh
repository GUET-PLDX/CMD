#!/usr/bin/env bash

set -euo pipefail

header=${1:-CMD.hpp}

if [[ ! -f "$header" ]]; then
  printf 'missing header: %s\n' "$header" >&2
  exit 1
fi

fail() {
  printf '%s\n' "$1" >&2
  exit 1
}

require_file_text() {
  local text=$1
  local message=$2
  grep -Fq "$text" "$header" || fail "$message"
}

forbid_file_text() {
  local text=$1
  local message=$2
  if grep -Fq "$text" "$header"; then
    fail "$message"
  fi
}

extract_block() {
  local start_pattern=$1
  awk -v start_pattern="$start_pattern" '
    { sub(/\r$/, "") }
    $0 ~ start_pattern { active = 1 }
    active {
      print
      opens = gsub(/{/, "{")
      closes = gsub(/}/, "}")
      depth += opens - closes
      if (opens > 0) {
        saw_open = 1
      }
      if (saw_open && depth == 0) {
        exit
      }
    }
  ' "$header"
}

require_block_text() {
  local block=$1
  local text=$2
  local message=$3
  grep -Fq "$text" <<<"$block" || fail "$message"
}

forbid_block_text() {
  local block=$1
  local text=$2
  local message=$3
  if grep -Fq "$text" <<<"$block"; then
    fail "$message"
  fi
}

require_order() {
  local block=$1
  shift
  local previous=0
  local text
  for text in "$@"; do
    local line
    line=$(grep -nF "$text" <<<"$block" | head -1 | cut -d: -f1)
    [[ -n "$line" ]] || fail "missing ordered statement: $text"
    ((line > previous)) || fail "incorrect statement order near: $text"
    previous=$line
  done
}

require_file_text '#include <atomic>' 'missing: atomic mode request support'
require_file_text '#include "semaphore.hpp"' 'missing: dispatch semaphore include'
require_file_text '#include "thread.hpp"' 'missing: dispatch thread include'
require_file_text 'uint64_t sequence = 0U;' 'missing: dispatch sequence'
require_file_text 'LibXR::Mutex mutex_;' 'missing: CMD mutex member'
require_file_text 'LibXR::MPMCQueue<DispatchSnapshot> dispatch_queue_' \
  'missing: dispatch snapshot queue'
require_file_text 'DISPATCH_QUEUE_CAPACITY};' \
  'missing: bounded dispatch queue capacity'
require_file_text 'LibXR::Semaphore dispatch_ready_;' \
  'missing: dispatch semaphore'
require_file_text 'LibXR::Thread dispatch_thread_;' 'missing: owner dispatch thread'
require_file_text 'std::atomic<uint32_t> mode_request_state_' \
  'missing: atomic latest mode request'
forbid_file_text 'MPMCQueue<Mode>' \
  'mode request queue can lose the replacement push when full'
require_file_text 'dispatch_task_stack_depth' \
  'missing: dispatch task stack constructor suffix'
require_file_text 'dispatch_thread_priority' \
  'missing: dispatch task priority constructor suffix'

feed_rc_default=$(extract_block 'void FeedRC\(const Data&')
feed_rc_source=$(extract_block 'void FeedRC\(RCInputSource')
feed_ai=$(extract_block 'void FeedAI\(const Data&')
set_mode=$(extract_block 'void SetCtrlMode\(Mode')
get_mode=$(extract_block 'Mode GetCtrlMode\(\)')
get_ai_status=$(extract_block 'bool GetAIGimbalStatus\(\)')
online=$(extract_block 'bool Online\(\)')
event_callback=$(extract_block 'auto callback =')
event_handler=$(extract_block 'void EventHandler\(uint32_t')
request_mode=$(extract_block 'void RequestCtrlMode\(Mode')
build_dispatch=$(extract_block 'DispatchSnapshot BuildDispatchLocked\(\)')
enqueue_dispatch=$(extract_block 'void EnqueueDispatchLocked\(const DispatchSnapshot&')
overflow_dispatch=$(extract_block 'void HandleDispatchOverflowLocked\(const DispatchSnapshot&')
take_dispatch=$(extract_block 'bool TakeNextDispatch\(DispatchSnapshot&')
dispatch_task=$(extract_block 'static void DispatchTask\(CMD\*')
publish=$(extract_block 'void Publish\(const DispatchSnapshot&')

forbid_block_text "$feed_rc_default" 'LockGuard' \
  'one-argument FeedRC must delegate before locking'
require_block_text "$feed_rc_default" \
  'FeedRC(RCInputSource::RC_INPUT_DR16, rc_data);' \
  'one-argument FeedRC must delegate to the source overload'

for block_name in feed_rc_source feed_ai set_mode get_mode get_ai_status online; do
  block=${!block_name}
  require_block_text "$block" 'LibXR::Mutex::LockGuard lock(mutex_);' \
    "missing lock: $block_name"
done

for block_name in feed_rc_source feed_ai; do
  block=${!block_name}
  require_block_text "$block" 'snapshot = BuildDispatchLocked();' \
    "missing snapshot build: $block_name"
  require_block_text "$block" 'EnqueueDispatchLocked(snapshot);' \
    "missing lock-ordered enqueue: $block_name"
  require_block_text "$block" 'dispatch_ready_.Post();' \
    "missing dispatcher wakeup: $block_name"
  forbid_block_text "$block" 'Publish(' \
    "feed must only enqueue, never publish: $block_name"
  if ! awk '
      /LibXR::Mutex::LockGuard lock\(mutex_\);/ { locked = 1 }
      locked && /EnqueueDispatchLocked\(snapshot\);/ { enqueued = 1 }
      enqueued && /^    }$/ { unlocked = 1 }
      /dispatch_ready_\.Post\(\);/ { exit unlocked ? 0 : 1 }
      END { if (!unlocked) exit 1 }
    ' <<<"$block"; then
    fail "enqueue must be lock-ordered and wakeup must follow unlock: $block_name"
  fi
done

require_block_text "$event_handler" \
  'RequestCtrlMode(static_cast<Mode>(event_id));' \
  'EventHandler must submit a non-blocking mode request'
forbid_block_text "$event_handler" 'SetCtrlMode' \
  'EventHandler must not call the locking setter'
forbid_block_text "$event_handler" 'LockGuard' 'EventHandler must not lock'
require_block_text "$event_callback" \
  'RequestCtrlMode(static_cast<Mode>(event_id));' \
  'Event callback must submit a non-blocking mode request'
forbid_block_text "$event_callback" 'SetCtrlMode' \
  'Event callback must not call the locking setter'
forbid_block_text "$event_callback" 'LockGuard' 'Event callback must not lock'
require_block_text "$request_mode" 'compare_exchange_weak' \
  'RequestCtrlMode must atomically retain the latest request'
forbid_block_text "$request_mode" 'LockGuard' 'RequestCtrlMode must not lock'
require_block_text "$build_dispatch" 'mode_request_state_.load' \
  'BuildDispatchLocked must apply the latest mode request'

require_block_text "$build_dispatch" 'snapshot.sequence = ++this->dispatch_sequence_;' \
  'BuildDispatchLocked must assign a monotonic dispatch sequence'
forbid_block_text "$build_dispatch" '.Publish(' \
  'BuildDispatchLocked must not publish topics'
forbid_block_text "$build_dispatch" 'cmd_event_.Active(' \
  'BuildDispatchLocked must not activate events'

require_block_text "$enqueue_dispatch" 'dispatch_queue_.Push(snapshot)' \
  'EnqueueDispatchLocked must push the ordered snapshot'
require_block_text "$enqueue_dispatch" 'HandleDispatchOverflowLocked(snapshot);' \
  'dispatch queue overflow must enter fail-safe handling'
require_block_text "$overflow_dispatch" 'overflow_pending_ = true;' \
  'dispatch overflow must latch a fail-safe snapshot'
require_block_text "$overflow_dispatch" \
  'overflow_snapshot_.event_id = CMD_EVENT_LOST_CTRL;' \
  'dispatch overflow must retain a LOST fail-safe event'
require_block_text "$overflow_dispatch" '++this->dispatch_overflow_count_;' \
  'dispatch overflow must increment its diagnostic counter'

require_block_text "$take_dispatch" 'LibXR::Mutex::LockGuard lock(mutex_);' \
  'dispatcher dequeue and overflow handoff must share state ordering'
require_block_text "$take_dispatch" 'dispatch_queue_.Pop(snapshot)' \
  'dispatcher must consume the ordered snapshot queue'
require_block_text "$take_dispatch" 'overflow_pending_' \
  'dispatcher must consume the latched fail-safe snapshot'
require_block_text "$dispatch_task" 'dispatch_ready_.Wait()' \
  'dispatch owner must wait for producer wakeups'
require_block_text "$dispatch_task" 'TakeNextDispatch(snapshot)' \
  'dispatch owner must drain queued snapshots'
require_block_text "$dispatch_task" 'cmd->Publish(snapshot);' \
  'dispatch owner must publish snapshots'
forbid_block_text "$dispatch_task" 'FeedRC(' \
  'dispatch owner must not feed CMD recursively'
forbid_block_text "$dispatch_task" 'FeedAI(' \
  'dispatch owner must not feed CMD recursively'

require_block_text "$publish" 'GimbalCMD gimbal = snapshot.gimbal;' \
  'Publish must copy the immutable gimbal snapshot'
require_block_text "$publish" 'ChassisCMD chassis = snapshot.chassis;' \
  'Publish must copy the immutable chassis snapshot'
require_block_text "$publish" 'LauncherCMD launcher = snapshot.launcher;' \
  'Publish must copy the immutable launcher snapshot'
require_order "$publish" 'gimbal_data_tp_.Publish(gimbal);' \
  'chassis_data_tp_.Publish(chassis);' 'fire_data_tp_.Publish(launcher);' \
  'cmd_event_.Active(snapshot.event_id);'
forbid_block_text "$publish" 'LockGuard' \
  'Publish must not hold the CMD state mutex'

if [[ ${CMD_SKIP_MUTATIONS:-0} != 1 ]]; then
  test_dir=$(cd "$(dirname "$0")" && pwd)
  tmp_dir=$(mktemp -d /tmp/cmd-concurrency-mutations.XXXXXX)
  trap 'rm -rf "$tmp_dir"' EXIT

  mutation_must_fail() {
    local name=$1
    local expression=$2
    local mutated="$tmp_dir/$name.hpp"
    sed "$expression" "$header" >"$mutated"
    if CMD_SKIP_MUTATIONS=1 bash "$test_dir/$(basename "$0")" "$mutated" \
      >/dev/null 2>&1; then
      fail "mutation survived: $name"
    fi
  }

  mutation_must_fail unlocked '/LockGuard lock(mutex_);/d'
  mutation_must_fail unordered_enqueue '/EnqueueDispatchLocked(snapshot);/d'
  mutation_must_fail missing_sequence \
    '/snapshot.sequence = ++this->dispatch_sequence_;/d'
  mutation_must_fail reentrant_publish \
    's/EnqueueDispatchLocked(snapshot);/Publish(snapshot);/'
  mutation_must_fail missing_overflow_latch '/overflow_pending_ = true;/d'
  mutation_must_fail missing_dispatch_publish '/cmd->Publish(snapshot);/d'
  mutation_must_fail lossy_mode_retry 's/compare_exchange_weak/store/'
fi

printf 'CMD concurrency static regression: PASS\n'
