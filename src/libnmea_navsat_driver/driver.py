# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser
from libnmea_navsat_driver.gps_conversions import LLtoUTM

class RosNMEADriver(object):
    def __init__(self):
        self.fix_pub = rospy.Publisher('global', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.pose_pub = rospy.Publisher('local', PoseWithCovarianceStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)

        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
        self.diag_pub_time = rospy.get_time()
        self.seq = 0
        self.invalid_cnt = 0
        self.fix_type = 'invalid'
        self.using_receiver_epe = False

        def set_fix_mapping(gga_quality, fix_status, epe_quality, fix_type):
            """Configure the properties for a fix quality from user-set params or a default.

            Lookup if the user set the quality_map for the value. Set it or default.
            Recall rosparam only supports string keys for dicts.
            """
            user_params = rospy.get_param('~quality_map/{0}'.format(gga_quality), {})

            if 'fix_status' in user_params:
                try:
                    fix_status = getattr(NavSatStatus, user_params['fix_status'])
                except AttributeError:
                    rospy.logwarn('User-defined NavSatStatus "{0}" for gga quality value "{1}" is an '
                                  'invalid status. Check sensor_msgs.msg.NavSatStatus for valid statuses.'
                                  .format(user_params['fix_status'], gga_quality))

            return (
                fix_status,
                user_params.get('epe_quality', epe_quality),
                user_params.get('fix_type', fix_type)
            )

        # Mappings of NavSatStatus, epe (estimated position error), and fix type to
        # NMEA GGA Fix Quality integers.
        self.fix_mappings = {
            0: set_fix_mapping(0, NavSatStatus.STATUS_NO_FIX, 1000000, 'no_fix'),
            1: set_fix_mapping(1, NavSatStatus.STATUS_FIX, 4.0, 'SPS'),
            2: set_fix_mapping(2, NavSatStatus.STATUS_SBAS_FIX, 0.1, 'DGPS'),
            4: set_fix_mapping(4, NavSatStatus.STATUS_GBAS_FIX, 0.02, 'RTK Fix'),
            5: set_fix_mapping(5, NavSatStatus.STATUS_GBAS_FIX, 4.0, 'RTK Float'),
            9: set_fix_mapping(9, NavSatStatus.STATUS_GBAS_FIX, 3.0, 'WAAS')
        }

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()

        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        
        current_pose_utm = PoseWithCovarianceStamped()
        current_pose_utm.header.stamp = current_time
        current_pose_utm.header.frame_id = frame_id

        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            self.seq = self.seq + 1
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            data = parsed_sentence['GGA']
            gps_qual = data['fix_type']

            try:
                # Unpack the fix params for this quality value
                current_fix.status.status, self.default_epe, self.fix_type = self.fix_mappings[gps_qual]
            except KeyError:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
                self.fix_type = 'Unknown'

            if gps_qual == 0:
                current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.using_receiver_epe = False
                self.invalid_cnt += 1

            current_fix.status.service = NavSatStatus.SERVICE_GPS
            current_fix.header.stamp = current_time

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = self.default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = self.default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = self.default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (hdop * self.alt_std_dev) ** 2  # FIXME

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            self.fix_pub.publish(current_fix)
            
            if not math.isnan(latitude) or not math.isnan(longitude):
                
                UTMNorthing, UTMEasting = LLtoUTM(latitude, longitude)[0:2]
                
                current_pose_utm.pose.pose.position.x = UTMEasting
                current_pose_utm.pose.pose.position.y = UTMNorthing
                current_pose_utm.pose.pose.position.z = altitude
            
                # Set orientation to 0; GPS doesn't provide orientation
                current_pose_utm.pose.pose.orientation.x = 0.0
                current_pose_utm.pose.pose.orientation.y = 0.0
                current_pose_utm.pose.pose.orientation.z = 0.0
                current_pose_utm.pose.pose.orientation.w = 0.0
                
                # Pose x/y/z covariance is whatever we decided h & v covariance is. 
                # Here is it the same as for ECEF coordinates
                current_pose_utm.pose.covariance[0] = (hdop * self.lon_std_dev) ** 2
                current_pose_utm.pose.covariance[7] = (hdop * self.lat_std_dev) ** 2
                current_pose_utm.pose.covariance[14] = (hdop * self.alt_std_dev) ** 2
                
                # Default angular pose is unknown
                current_pose_utm.pose.covariance[21] = 1e03
                current_pose_utm.pose.covariance[28] = 1e03
                current_pose_utm.pose.covariance[35] = 1e03
            
                self.pose_pub.publish(current_pose_utm)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                self.time_ref_pub.publish(current_time_ref)

            if (self.diag_pub_time < rospy.get_time()):
                self.diag_pub_time += 1
                diag_arr = DiagnosticArray()
                diag_arr.header.stamp = rospy.get_rostime()
                diag_arr.header.frame_id = frame_id
                diag_msg = DiagnosticStatus()
                diag_msg.name = 'GPS_status'
                diag_msg.hardware_id = 'GPS'
                diag_msg.level = DiagnosticStatus.OK
                diag_msg.message = 'Received GGA Fix'
                diag_msg.values.append(KeyValue('Sequence number', str(self.seq)))
                diag_msg.values.append(KeyValue('Invalid fix count', str(self.invalid_cnt)))
                diag_msg.values.append(KeyValue('Latitude', str(latitude)))
                diag_msg.values.append(KeyValue('Longitude', str(longitude)))
                diag_msg.values.append(KeyValue('Altitude', str(altitude)))
                diag_msg.values.append(KeyValue('GPS quality', str(gps_qual)))
                diag_msg.values.append(KeyValue('Fix type', self.fix_type))
                diag_msg.values.append(KeyValue('Number of satellites', str(data['num_satellites'])))
                diag_msg.values.append(KeyValue('Receiver providing accuracy', str(self.using_receiver_epe)))
                diag_msg.values.append(KeyValue('Hdop', str(hdop)))
                diag_msg.values.append(KeyValue('Latitude std dev', str(hdop * self.lat_std_dev)))
                diag_msg.values.append(KeyValue('Longitude std dev', str(hdop * self.lon_std_dev)))
                diag_msg.values.append(KeyValue('Altitude std dev', str(hdop * self.alt_std_dev)))
                diag_arr.status.append(diag_msg)
                self.diag_pub.publish(diag_arr)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']

        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            """Add the TF prefix"""
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
