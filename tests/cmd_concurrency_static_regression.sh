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

require_file_text 'LibXR::Mutex mutex_;' 'missing: CMD mutex member'
require_file_text 'LibXR::MPMCQueue<Mode> mode_requests_{4};' \
  'missing: mode request queue'
require_file_text 'struct DispatchSnapshot {' 'missing: dispatch snapshot'

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
  require_block_text "$block" 'Publish(snapshot);' \
    "missing snapshot publish: $block_name"
  if ! awk '
      /LibXR::Mutex::LockGuard lock\(mutex_\);/ { locked = 1 }
      locked && /^    }$/ { unlocked = 1 }
      /Publish\(snapshot\);/ { exit unlocked ? 0 : 1 }
      END { if (!unlocked) exit 1 }
    ' <<<"$block"; then
    fail "Publish must follow the lock scope: $block_name"
  fi
done

require_block_text "$event_handler" 'RequestCtrlMode(static_cast<Mode>(event_id));' \
  'EventHandler must enqueue a mode request'
forbid_block_text "$event_handler" 'SetCtrlMode' \
  'EventHandler must not call the locking setter'
forbid_block_text "$event_handler" 'LockGuard' \
  'EventHandler must not lock'
require_block_text "$event_callback" \
  'RequestCtrlMode(static_cast<Mode>(event_id));' \
  'Event callback must enqueue a mode request'
forbid_block_text "$event_callback" 'SetCtrlMode' \
  'Event callback must not call the locking setter'
forbid_block_text "$event_callback" 'LockGuard' \
  'Event callback must not lock'
require_block_text "$request_mode" 'mode_requests_.Push' \
  'RequestCtrlMode must push to the MPMC queue'
forbid_block_text "$request_mode" 'LockGuard' \
  'RequestCtrlMode must not lock'

require_block_text "$build_dispatch" 'mode_requests_.Pop' \
  'BuildDispatchLocked must drain mode requests'
forbid_block_text "$build_dispatch" '.Publish(' \
  'BuildDispatchLocked must not publish topics'
forbid_block_text "$build_dispatch" 'cmd_event_.Active(' \
  'BuildDispatchLocked must not activate events'

require_block_text "$publish" 'GimbalCMD gimbal = snapshot.gimbal;' \
  'Publish must copy the immutable gimbal snapshot'
require_block_text "$publish" 'ChassisCMD chassis = snapshot.chassis;' \
  'Publish must copy the immutable chassis snapshot'
require_block_text "$publish" 'LauncherCMD launcher = snapshot.launcher;' \
  'Publish must copy the immutable launcher snapshot'
require_block_text "$publish" 'gimbal_data_tp_.Publish(gimbal);' \
  'Publish must send the gimbal snapshot'
require_block_text "$publish" 'chassis_data_tp_.Publish(chassis);' \
  'Publish must send the chassis snapshot'
require_block_text "$publish" 'fire_data_tp_.Publish(launcher);' \
  'Publish must send the launcher snapshot'
require_block_text "$publish" 'cmd_event_.Active(snapshot.event_id);' \
  'Publish must activate the snapshot event'
forbid_block_text "$publish" 'LockGuard' 'Publish must not lock the CMD mutex'

printf 'CMD concurrency static regression: PASS\n'
