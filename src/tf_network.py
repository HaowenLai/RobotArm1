#!/usr/bin/python
import tensorflow as tf
import numpy as np

save_file = '/home/savage/workspace/cpp_ws/Aruco-marker/data/model.ckpt-300000'
inNum  = 2
outNum = 2

def addLayer(inputData,inSize,outSize,activity_function = None):
    Weights = tf.Variable(tf.truncated_normal([inSize,outSize],stddev=0.1))
    basis = tf.Variable(tf.zeros([1,outSize]))
    weights_plus_b = tf.matmul(inputData,Weights)+basis
    if activity_function is None:
        ans = weights_plus_b
    else:
        ans = activity_function(weights_plus_b)
    return ans

xs = tf.placeholder(tf.float32,[None,inNum])

l1 = addLayer(xs,inNum,20,activity_function=tf.nn.relu)
l2 = addLayer(l1,20,20,activity_function=tf.nn.relu)
l3 = addLayer(l2,20,10,activity_function=tf.nn.relu)
l4 = addLayer(l3,10,outNum,activity_function=None)
l5 = tf.round(l4)

init = tf.global_variables_initializer()
sess = tf.Session()
sess.run(init)

saver = tf.train.Saver()
saver.restore(sess, save_file)

#main function, interface for cpp program
def main(coordinate):
    print('enter main successfully')
    coordinate = [coordinate]
    motor_value = sess.run(l5,feed_dict={xs:coordinate})
    
    result = tuple(motor_value[0])
    
    return result

