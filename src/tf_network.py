#!/usr/bin/python3
import tensorflow as tf
import numpy as np
import os

os.environ["CUDA_VISIBLE_DEVICES"] = "1"

# --------------------------- For position Network --------------
save_file = '/home/savage/workspace/cpp_ws/Aruco-marker/data/model.ckpt-100000'
inNum  = 2
outNum = 2

def addLayer(inputData,inSize,outSize,kp,activity_function = None):
    Weights = tf.Variable(tf.truncated_normal([inSize,outSize],stddev=0.1))
    basis = tf.Variable(tf.zeros([1,outSize]))
    weights_plus_b = tf.matmul(inputData,Weights)+basis
    #add dropout here
    weights_plus_b = tf.nn.dropout(weights_plus_b,kp)
    if activity_function is None:
        ans = weights_plus_b
    else:
        ans = activity_function(weights_plus_b)
    return ans

xs = tf.placeholder(tf.float32,[None,inNum])
keep_prob = tf.placeholder(tf.float32)

l1 = addLayer(xs,inNum,20,keep_prob,activity_function=tf.nn.relu) # relu是激励函数的一种  
l2 = addLayer(l1,20,20,keep_prob,activity_function=tf.nn.relu)
l3 = addLayer(l2,20,20,keep_prob,activity_function=tf.nn.relu)
l3 = addLayer(l3,20,20,keep_prob,activity_function=tf.nn.relu)
l3 = addLayer(l3,20,20,keep_prob,activity_function=tf.nn.relu)
l3 = addLayer(l3,20,20,keep_prob,activity_function=tf.nn.relu)
l3 = addLayer(l3,20,20,keep_prob,activity_function=tf.nn.relu)
l4 = addLayer(l3,20,2,keep_prob,activity_function=None)
l5 = tf.round(l4)

init = tf.global_variables_initializer()
sess = tf.Session()
sess.run(init)

saver = tf.train.Saver()
saver.restore(sess, save_file)

#main function, interface for cpp program
def bp_main(coordinate):
    print('enter main successfully')
    coordinate = [coordinate]
    motor_value = sess.run(l5,feed_dict={xs:coordinate,keep_prob:1})
    
    result = tuple(motor_value[0])
    
    return result
