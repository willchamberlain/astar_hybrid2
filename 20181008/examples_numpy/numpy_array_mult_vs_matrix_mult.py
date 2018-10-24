#!/usr/bin/env python

# This is server.py file

import sys

import numpy as np


print "Pre-start: check numpy"

A = np.array ((
    (8,     1,     6),
    (3,     5,     7),
    (4,     9,     2)
    ), dtype=np.float64)
print "A as np.array"    
print A
B = np.array ((    
    (6.4000 ,   2.4000 ,   3.2000),
    (0.8000 ,   4.0000 ,   7.2000),
    (4.8000 ,   5.6000 ,   1.6000)
    ), dtype=np.float64)    
print "B as np.array"    
print B
print "--------"

print "np.multiply(A,B)  :  numpy np.array 'multiply' is _array_ multiplication, not matrix multiplication"
print np.multiply(A,B)

print "np.multiply(B,A)  :  numpy np.array 'multiply' is _array_ multiplication, not matrix multiplication"
print np.multiply(B,A)
print "--------"

print "np.dot(A,B)  :  numpy np.array 'dot' is 'vectorized'/matrix multiplication"
print np.dot(A,B)

print "np.dot(B,A)  :  numpy np.array 'dot' is 'vectorized'/matrix multiplication"
print np.dot(B,A)
print "--------"


A = np.matrix ((
    (8,     1,     6),
    (3,     5,     7),
    (4,     9,     2)
    ), dtype=np.float64)
print "A as np.matrix : np.matrix restricted to 2D"    
print A
B = np.matrix ((    
    (6.4000 ,   2.4000 ,   3.2000),
    (0.8000 ,   4.0000 ,   7.2000),
    (4.8000 ,   5.6000 ,   1.6000)
    ), dtype=np.float64)    
print "B as np.matrix : np.matrix restricted to 2D"    
print B
print "--------"
print "A*B  :  numpy np.matrix '*' is 'vectorized'/matrix multiplication/matrix product"
print A*B

print "B*A  :  numpy np.matrix '*' is 'vectorized'/matrix multiplication/matrix product"
print B*A
print "--------"

