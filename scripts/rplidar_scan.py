#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, ChannelFloat32
from std_msgs.msg import String
from numpy import concatenate
from copy import deepcopy
from numpy import array as nparray
from math import pi, radians
from pooling import *
from resample import *
import time


class RPLIDAR_SCAN:
    def __init__(self, name='RPLIDAR_SCAN', autostart=True, rate=11, log_level=rospy.INFO):
        self.name = name
        self.rospy = rospy
        self.rospy.init_node(self.name, anonymous=True, log_level=log_level)
        self.rospy.loginfo("[%s] Starting Node ", self.name)
        self.rate = self.rospy.Rate(rate)

        self.initParams()
        self.initVariables()
        self.initPublishers()
        self.initSubscribers()

        if autostart:
            self.start()

    def initParams(self):
        self.lidar_intensities = float(rospy.get_param('rplidar_params/intensities'))
        self.output_samples_length = rospy.get_param('rplidar_params/output_samples')
        self.reduction_method = str(rospy.get_param('rplidar_params/reduction_method')).lower()
        self.lidar_range = dict(rospy.get_param('rplidar_params/range'))
        self.lidar_fov = int(rospy.get_param('rplidar_params/fov'))
        self.lidar_rotation = int(rospy.get_param('rplidar_params/rotation'))
        self.mask_value = inf
        self.reduction = None
        self.length_ratio = None
        self.reduction_position = None
        self.lidar_raw_ranges_length = None
        self.reduce_fixed_params = None

        if self.lidar_intensities < 0:
            self.rospy.logerr(f'[{self.name}] lidar_intensities cant admit negative values')
            raise TypeError(f'[{self.name}] lidar_intensities cant admit negative values')

        try:
            if self.output_samples_length < 0:
                self.rospy.logerr(f'[{self.name}] output_samples_length cant admit negative values')
                raise TypeError(f'[{self.name}] output_samples_length cant admit negative values')
        except TypeError or ValueError:
            self.rospy.logwarn(f'[{self.name}] [output_samples] not informed or invalid, value will be set to [raw_samples_length] and [reduction_method] set to [None]')
            self.output_samples_length = None
            self.reduction_method = None

        if self.lidar_range['min'] < 0 or self.lidar_range['max'] < 0:
            self.rospy.logerr(f'[{self.name}] lidar_range cant admit negative values')
            raise TypeError(f'[{self.name}] lidar_range cant admit negative values')
        if self.lidar_fov < 0 or self.lidar_fov > 360:
            self.rospy.logerr(f'[{self.name}] lidar_fov only admit values between 0~360')
            raise TypeError(f'[{self.name}] lidar_fov only admit values between 0~360')
        if self.lidar_rotation < -180 or self.lidar_rotation > 180:
            self.rospy.logerr(f'[{self.name}] lidar_rotation only admit values between -180~180')
            raise TypeError(f'[{self.name}] lidar_rotation only admit values between -180~180')

    def defineParams(self, range_samples):
        self.lidar_raw_ranges_length = len(range_samples)
        self.rospy.loginfo(
            f'[{self.name}] ##### rplidar samples read! [{self.lidar_raw_ranges_length}] raw samples! #####')

        # auto set samples if None
        if not self.output_samples_length:
            self.output_samples_length = self.lidar_raw_ranges_length
        self.length_ratio = self.lidar_raw_ranges_length / self.output_samples_length

        if not (0 < self.lidar_fov < 360) or not self.output_samples_length:
            self.min_mask_index, self.max_mask_index = None, None
            self.mask_values_array = nparray(tuple())
        else:
            # define indexes to mask method
            self.min_mask_index = round(self.output_samples_length * (radians(self.lidar_fov / (4 * pi))))
            self.max_mask_index = round(self.output_samples_length * (1 - radians(self.lidar_fov / (4 * pi))))
            self.mask_values_array = nparray(int(self.max_mask_index - self.min_mask_index) * [self.mask_value])

        # this section define parameters and function to match selected reduction method
        if not self.reduction_method or 'none' in self.reduction_method:
            # não estou conseguindo chavear esses parametros ao carregá-los então estou modificando aqui. Ass: AJ
            self.reduction_method = 'none' #8,54% de erro em 2000 amostras com 6% de corte emax:10.52A1
            # self.reduction_method = 'resample' #8,95% de erro em 2000 amostras com 6% de corte emax:9.44A16
            # self.reduction_method = 'pooling' # não consegui validar
            
            
        else:
            if not('pooling' in self.reduction_method or 'resample' in self.reduction_method):
                self.rospy.logerr(f'[{self.name}] reduction_method only admit "[max/min/mean/(int)]-pooling", "resample", or "None"')
                raise TypeError(f'[{self.name}] reduction_method only admit "[max/min/mean/(int)]-pooling", "resample", or "None"')

            try:
                # resample method
                if 'resample' in self.reduction_method:
                    self.reduction = resample
                    self.reduce_fixed_params = tuple([self.output_samples_length])

                # cases bellow refer to max/min/mean pooling method
                elif 'max' in self.reduction_method:
                    self.reduction = max_pooling
                    self.reduce_fixed_params = (self.length_ratio, self.lidar_raw_ranges_length)
                elif 'min' in self.reduction_method:
                    self.reduction = min_pooling
                    self.reduce_fixed_params = (self.length_ratio, self.lidar_raw_ranges_length)
                elif 'mean' in self.reduction_method:
                    self.reduction = mean_pooling
                    self.reduce_fixed_params = (self.length_ratio, self.lidar_raw_ranges_length)
                    print("POOOOLING MEDIO")

                # next case refer to position pooling method
                elif type(int(self.reduction_method.split('-')[0])) is int:
                    self.reduction = position_pooling
                    self.reduction_position = int(self.reduction_method.split('-')[0])
                    self.reduction_sections = nparray(range(0, self.lidar_raw_ranges_length, int(self.length_ratio)))
                    self.reduce_fixed_params = (self.reduction_position, self.reduction_sections)

                else:
                    raise TypeError

            except TypeError:
                TypeError(f'[{self.name}] pooling reduction method need [max/min/mean/(int)] information')

        self.rospy.loginfo(f'[{self.name}] * PARAMETERS *\n'
                           f' * fov: [{self.lidar_fov}]\n'
                           f' * min_mask_index: [{self.min_mask_index}]\n'
                           f' * max_mask_index: [{self.max_mask_index}]\n'
                           f' * output_samples_length: [{self.output_samples_length}]\n'
                           f' * fov_samples(length): [{self.output_samples_length - len(self.mask_values_array)}]\n'
                           f' * mask_samples(length): [{len(self.mask_values_array)}]\n'
                           f' * reduction_method: {self.reduction_method}')

        self.lidar_redata.angle_min = -pi + radians(self.lidar_rotation)
        self.lidar_redata.angle_max = pi + radians(self.lidar_rotation)
        self.lidar_redata.angle_increment = 2 * pi / self.output_samples_length
        self.lidar_redata.time_increment = self.lidar_raw_msg.time_increment * self.length_ratio

        self.lidar_redata.range_min = self.lidar_range['min']
        self.lidar_redata.range_max = self.lidar_range['max']
        #self.lidar_redata.intensities = nparray(self.output_samples_length * [self.lidar_intensities])
        self.lidar_redata.intensities = self.lidar_intensities

        if self.reduction_position:
            if self.length_ratio % 1 > 0:
                self.rospy.logerr(
                    f'[{self.name}] input_array length [{self.lidar_raw_ranges_length}] % pooling value [{self.length_ratio}] > 0')
                raise TypeError(f'[{self.name}] input_array length % pooling value > 0')
            if self.reduction_position > self.length_ratio:
                self.rospy.logerr(
                    f'[{self.name}] reduction_position [{self.reduction_position}] > length_ratio value [{self.length_ratio}]')
                raise TypeError(
                    f'[{self.name}] reduction_position [{self.reduction_position}] > length_ratio value [{self.length_ratio}]')

    def initVariables(self):
        self.lidar_raw_msg = LaserScan()
        self.lidar_redata = LaserScan()
        self.callback_flag = False
        self.running_flag = False
        self.reading_params_flag = False
        self.ready_flag = False
        self.length_error_flag = False

    def initPublishers(self):
        self.lidar_pub = self.rospy.Publisher('/scan_unfiltered', LaserScan, queue_size=15)

    def initSubscribers(self):
        self.lidar_sub = self.rospy.Subscriber("/raw_scan", LaserScan, self.lidarCallback, queue_size=15)
        self.lidar_params_sub = self.rospy.Subscriber('/scan_params', String, self.lidarParamsCallback, queue_size=1)

    def lidarCallback(self, msg):
        if any(msg.ranges):
            self.lidar_raw_msg = msg
            self.lidar_raw_ranges = deepcopy(self.lidar_raw_msg.ranges)
            self.callback_flag = True
        if any(msg.intensities):
            self.lidar_intensities = msg.intensities

    def lidarParamsCallback(self, msg):
        # this function update parameters thought a string message callback
        # parameters and variables are reloaded and the /scan change dynamically
        if msg:
            params = msg.data.split(' ')
            if '-i' in params:  # 'intensities'
                self.rospy.set_param('rplidar_params/intensities', float(params[1 + params.index('-i')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/intensities" set to "{self.rospy.get_param("rplidar_params/intensities")}"')
            if '-s' in params:  # 'samples'
                self.rospy.set_param('rplidar_params/output_samples', (params[1 + params.index('-s')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/output_samples" set to "{self.rospy.get_param("rplidar_params/output_samples")}"')
            if '-m' in params:  # 'reduction method'
                self.rospy.set_param('rplidar_params/reduction_method', str(params[1 + params.index('-m')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/reduction_method" set to "{self.rospy.get_param("rplidar_params/reduction_method")}"')
            if '-min' in params:  # 'range["min"]'
                self.rospy.set_param('rplidar_params/range/min', float(params[1 + params.index('-min')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/range/min" set to "{self.rospy.get_param("rplidar_params/range/min")}"')
            if '-max' in params:  # 'range["max"]'
                self.rospy.set_param('rplidar_params/range/max', float(params[1 + params.index('-max')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/range/max" set to "{self.rospy.get_param("rplidar_params/range/max")}"')
            if '-f' in params:  # 'fov'
                self.rospy.set_param('rplidar_params/fov', int(params[1 + params.index('-f')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/fov" set to "{self.rospy.get_param("rplidar_params/fov")}"')
            if '-r' in params:  # 'rotation'
                self.rospy.set_param('rplidar_params/rotation', int(params[1 + params.index('-r')]))
                self.rospy.loginfo(f'[{self.name}] "rplidar_params/rotation" set to "{self.rospy.get_param("rplidar_params/rotation")}"')

            self.softReload()

    def start(self):
        self.running_flag = True
        self.main()

    def softReload(self):
        self.rospy.logwarn(f'[{self.name}] waiting end of current loop to reload, delay for first /scan publish expected')
        # while self.callback_flag:
        #     continue
        self.running_flag = False
        self.initVariables()
        self.initParams()
        self.start()

    def main(self):
        while not self.rospy.is_shutdown():
            if not self.callback_flag or not self.running_flag:
                # commit this sleep to achieve minimal delay, but maybe overrate comparisons
                # self.rate.sleep()
                continue

            # setup parameters on start every reload
            if not self.reading_params_flag and not self.ready_flag:
                self.reading_params_flag = True
                self.defineParams(self.lidar_raw_ranges)
                self.ready_flag = True
                # this section only will run again if a problem occur
                # proceed to error detection, reduction, mask and publish looping

            if not self.ready_flag:
                self.rate.sleep()
                continue

            # this section prevent error related to inconsistent raw sampling number
            if self.length_ratio and self.output_samples_length > self.lidar_raw_ranges_length and not self.length_error_flag:
                self.output_samples_length = self.lidar_raw_ranges_length
                self.length_ratio, self.reduction_position = 1, 1

                self.rospy.logwarn(
                    f'[{self.name}] output_samples_length is bigger then raw_samples, '
                    f'[{self.name}] output_samples_length set to max_value for this method [{self.lidar_raw_ranges_length}]')
                self.rate.sleep()
                continue

            # this section recover from error related to inconsistent raw sampling number
            if self.output_samples_length == self.lidar_raw_ranges_length and self.length_error_flag:
                self.length_error_flag = False
                self.output_samples_length = int(rospy.get_param('rplidar_params/output_samples'))
                self.rospy.logwarn(f'[{self.name}] output_samples_length returned to user defined length [{self.output_samples_length}]')
                self.softReload()
                self.rate.sleep()
                continue

            # synchronize frame updates for mapping and navigation
            self.lidar_redata.header = self.lidar_raw_msg.header

            # reduce and mask out fov values
            if self.reduction:
                reduced_ranges = self.reduction(nparray(self.lidar_raw_ranges), *self.reduce_fixed_params)
                self.lidar_redata.ranges = concatenate((reduced_ranges[0:self.min_mask_index], self.mask_values_array, reduced_ranges[self.max_mask_index:-1]))
            elif any(self.mask_values_array):
                self.lidar_redata.ranges = concatenate((self.lidar_raw_ranges[0:self.min_mask_index], self.mask_values_array, self.lidar_raw_ranges[self.max_mask_index:-1]))
            else:
                self.lidar_redata.ranges = self.lidar_raw_msg.ranges

            # mask range
            for sample_index in range(len(self.lidar_redata.ranges)):
                if not (self.lidar_range['min'] < self.lidar_redata.ranges[sample_index] < self.lidar_range['max']):
                    self.lidar_redata.ranges[sample_index] = inf

            # publish section
            # self.lidar_pub.publish(self.lidar_redata)

            try:
                # if time.time()%1 >0.9: 
                #     self.rospy.logerr("overflow error")
                #     self.softReload()
                #     continue
                self.lidar_pub.publish(self.lidar_redata)
            except OverflowError:   # probably this error is associated with high main rate
                self.rospy.logerr(f'[{self.name}] OverflowError: float too large to pack with f format')
                self.softReload()
                continue

            # sweet dreams
            self.rate.sleep()
            self.callback_flag = False
        self.__exit__()

    def __exit__(self):
        quit()


if __name__ == '__main__':
    try:
        rplidar_scan = RPLIDAR_SCAN('RPLIDAR_SCAN')
    except rospy.ROSInterruptException or KeyboardInterrupt:
        quit()
