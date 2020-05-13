#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   DeepLearning_DetectCase.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/2/17 20:46   msliu      1.0      

@Description
------------
通过加载已经训练好的模型，正向预测情况
1.获取数据（IR image and Lidar Data）
2.构建神经网络
3.加载variables 并且 载入数据
4.输出判断情况
"""
import os
import sys
import numpy as np
import tensorflow as tf

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
print("(Deep Learning class)father path:", father_path)


class DeepLearningDetectCase(object):
    def __init__(self, model_name, class_num=6, sample_num=1, strides=1):
        # 分类数量 3 或 4，看包不包含原地不动的情况
        self.class_num = class_num
        # sample的数量：连续几帧作为一个输入
        self.sample_num = sample_num
        # 步进的距离
        self.strides = strides

        # model path
        self.model_path = father_path + os.path.sep + "resource" + os.path.sep + "model" + os.path.sep + model_name

        # 输入数据 定义的格式
        self.ir_data = np.ones(32 * 24 * self.sample_num).reshape([32 * 24, self.sample_num])
        self.legs_position = np.ones(4 * self.sample_num).reshape([4, self.sample_num])

        # tensorflow的数据入口（占位符）
        self.inputs = tf.placeholder(tf.float32, shape=[None, 24 * 32 + 4, self.sample_num], name="inputs")
        self.labels = tf.placeholder(tf.int32, shape=[None], name="labels")

        height = 24
        width = 32
        leg_size = 4
        self.flat_size = (height * width + leg_size) * self.sample_num
        # tensorflow的variables
        self.fc1_weights = tf.get_variable('fc1_weights', shape=[self.flat_size, 512], dtype=tf.float32)
        self.fc1_biases = tf.get_variable('f1_biases', shape=[512], dtype=tf.float32)
        self.fc2_weights = tf.get_variable('fc2_weights', shape=[512, 512], dtype=tf.float32)
        self.fc2_biases = tf.get_variable('f2_biases', shape=[512], dtype=tf.float32)
        self.fc3_weights = tf.get_variable('fc3_weights', shape=[512, self.class_num], dtype=tf.float32)
        self.fc3_biases = tf.get_variable('f3_biases', shape=[self.class_num], dtype=tf.float32)

        # 构建模型并且把结果保存在predict_result中
        self.predict_result, self.logits = self.build_mode()
        self.sess = tf.Session()
        saver = tf.train.Saver()
        print("DL model path:",self.model_path)
        saver.restore(self.sess, self.model_path)
        self.dict_class = {0: "left_left",
                           1: "left_forward",
                           2: "left_right",
                           3: "right_left",
                           4: "right_forward",
                           5: "right_right"}
        pass

    def build_mode(self):
        net = self.inputs
        net = tf.reshape(net, shape=[-1, self.flat_size])
        net = tf.nn.relu(tf.add(tf.matmul(net, self.fc1_weights), self.fc1_biases))
        net = tf.nn.relu(tf.add(tf.matmul(net, self.fc2_weights), self.fc2_biases))
        net = tf.add(tf.matmul(net, self.fc3_weights), self.fc3_biases)
        logits = tf.nn.softmax(net)
        classes = tf.cast(tf.argmax(logits, axis=1), dtype=tf.int32)
        return classes, logits

    def obtain_input(self, ir_data_sample, leg_sample):
        np_size_32 = np.ones(32 * 24).reshape([32 * 24, 1])
        max_colum = ir_data_sample.max(axis=1).reshape([ir_data_sample.shape[0], 1])
        max_value = max_colum.max()
        min_colum = ir_data_sample.min(axis=1).reshape([ir_data_sample.shape[0], 1])
        min_value = min_colum.min()
        min_np = np_size_32 * min_value

        max_minus_min = max_value - min_value
        normal_ir = (ir_data_sample - min_np) / max_minus_min  # normal_ir shape[list_num,32*24]

        # legs_position normalization
        max_leg = leg_sample.max(axis=0)
        min_leg = leg_sample.min(axis=0)
        if max_leg == min_leg:
            max_minus_min = 1
        else:
            max_minus_min = max_leg - min_leg
        normal_leg = (leg_sample - min_leg) / max_minus_min

        input_data = np.r_[normal_ir, normal_leg]
        return {self.inputs: input_data.reshape([1, 24 * 32 + 4, self.sample_num])}

    def print_predict_result(self, ir_data_sample, leg_sample):
        result_, logits_ = self.sess.run([self.predict_result, self.logits],
                                         feed_dict=self.obtain_input(ir_data_sample, leg_sample))

        print("--------------", self.dict_class[int(result_)], "--------------------")
        return result_
