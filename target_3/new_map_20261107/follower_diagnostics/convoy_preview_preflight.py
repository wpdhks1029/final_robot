"""재부팅 직후 DDS 발견 대기. 중복 발행자 금지 조건은 유지한다."""
import time

OUTPUTS=('/follower/cmd_vel_test','/follower/cmd_vel_guard_test',
         '/follower/cmd_vel_stamped_test','/follower/cmd_vel_guard_stamped_test')
INPUTS=('/leader/odom','/leader/path','/follower/odom_map','/follower/scan')


def wait_for_inputs(node,spin,*,timeout=12.,clock=time.monotonic):
    end=clock()+timeout;counts={}
    while clock()<end:
        spin()
        for topic in OUTPUTS:
            if node.get_publishers_info_by_topic(topic):
                raise RuntimeError('Preview already running: '+topic+'; do not duplicate')
        counts={topic:len(node.get_publishers_info_by_topic(topic)) for topic in INPUTS}
        if any(count>1 for count in counts.values()):
            raise RuntimeError('Duplicate input publishers: '+str(counts))
        if all(count==1 for count in counts.values()):return counts
    raise RuntimeError('Input publisher discovery timed out: '+str(counts))
