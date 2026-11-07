# 실제 구동 금지 상태의 시리얼 오류 조사

## 사용자 실제 출력

- PID2961이MD200T포트단독소유,mode HARDWARE_CANDIDATE/input_kind guard。
- armed=false/enabled=false/accepted_commands=0/rejected_commands=0。
- valid_feedback=100 뒤 SERIAL_WRITE_ERROR/io_failed=true/fresh=false。
  measured_rpm=[0,0]은과거최종측정이며오류후현재정지측정이아님。
- gateway는NOT_ARMED_OR_NOT_READY로전달차단정상。
  신규gateway의12초탐색대기로발행자발견성공,이전탐색오류는현재미재현。
- 접근가능한 /var/log/kern.log에서오늘부팅시각USBattach는보이나
  해당오류시점의USB분리/under-voltage흔적은확인하지못함。
  로그지연/누락가능성이있어USB문제를배제한것은아님。

## 진단 보강

- 기존core는예외내용을버리고일반SERIAL_WRITE_ERROR만저장했음。
  실제write뿐아니라preempt전input buffer reset예외도같은분류라원인분리불가。
- last_io_error에operation/type/message/packet/expected/written정보보존추가。
  first에러후best-effortSTOP실패로원래오류가덮이지않음。
- status에last_io_error/feedback_age_s추가,처음fault발생시ROSerror로그출력。
- 타이밍/timeout/RPM상한/arm정책은변경없음,자동port재연결안함。
  재시작전까지io_failedlatched유지。
- 진단4/기존core39/drive11/gateway9=63개모의검사통과。
- 이미실행중인PID2961은옛코드이므로기존원본예외를소급확인할수없음。
  정확한이전예외를알려면종전driver로그가별도로있어야하며현재출력에는없음。

## 다음 단계

전달부/센서/EKF유지,실제driver만정확히종료/포트해제후
동일구동금지옵션으로재시작。최소15초유효피드백증가/새오류정보확인。
enable-motion-input/arm호출/USB분리/설정쓰기/임의timeout연장안함。
