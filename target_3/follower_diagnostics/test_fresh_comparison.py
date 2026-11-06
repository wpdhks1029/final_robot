import unittest
from compare_fresh_ekf import convert


def row(topic,x='0'):
    return dict(topic=topic,stamp_ns='1000000000',rx_s='0.1',x=x,y='0',vx='.02',wz='0')


class Tests(unittest.TestCase):
    def test_select_and_remap_only_offline(self):
        rows=[row('/follower/odom_raw','1'),row('/follower/odom_position_test','2'),row('/follower/imu/data')]
        a=convert(rows,'/follower/odom_position_test')
        self.assertEqual(len(a),2);self.assertEqual(a[0]['x'],'2')
        self.assertEqual(a[0]['topic'],'/follower/odom_raw')
        self.assertEqual(a[0]['frame'],'follower/odom')
        self.assertEqual(rows[1]['topic'],'/follower/odom_position_test')

    def test_missing_source_refused(self):
        with self.assertRaises(ValueError):convert([row('/follower/imu/data')],'/missing')


if __name__=='__main__':unittest.main()
