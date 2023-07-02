#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import os
import numpy as np
from CameraCalibrator import Calibrator
import matplotlib.pyplot as plt
import copy

class BirdEyeView() :
	def __init__(self) :
		self.__left = [-150,480] ## width, height
		self.__right = [2070,480]
		self.__left_top = [890,230]
		self.__right_top = [1030,230] 

		self.__src = np.float32([self.__left, self.__left_top, self.__right_top, self.__right]) 
		self.__dst = np.float32([[100,480],[100,0],[1820,0],[1820,480]]) 

	def setROI(self,frame) :
		#print('frame shape = ',frame.shape)
		self.__roi = np.array([[self.__left,self.__left_top,self.__right_top,self.__right]]).astype(np.int32)
		return cv2.polylines(frame, np.int32(self.__roi),True,(255,0,0),10)## 10 
		#return self.__roi

	def warpPerspect(self,frame) :
		y = frame.shape[0]
		x = frame.shape[1]
		M = cv2.getPerspectiveTransform(self.__src,self.__dst) ## 
		return cv2.warpPerspective(frame, M,(x,y), flags=cv2.INTER_LINEAR) ##

	def extractChannel(self, img, color_space, th, channel = 0) : ##
		color_space = cv2.cvtColor(img,color_space)
		extracted = color_space[:,:,channel]
		output = np.zeros_like(extracted)
		output[(extracted >= th[0]) & (extracted <= th[1])]
		return output

	def combineAllColorChannel(self, warped_img, color_th, sobel_th) :
		"""
		모든 색상 공간에서 색상을 추출해서 합침
		"""
		s_channel = self.extractChannel(warped_img,cv2.COLOR_BGR2HLS,color_th,2)
		l_channel = self.extractChannel(warped_img,cv2.COLOR_BGR2HLS,color_th,1)
		y_channel= self.extractChannel(warped_img,cv2.COLOR_BGR2YUV,color_th,0)

		sobel_x = self.Sobel(warped_img, sobel_th, 'x')
		sobel_dir= self.Sobel(warped_img, [0.7,25], 'dir')
		
		output = np.zeros_like(s_channel)
		output[(((s_channel == 1) & (l_channel==1)) & (y_channel==1)) | (sobel_x == 1)  ] = 1

		return output

	def Sobel(self,warped_img, th, sobel_type, kernel_size=3) :
		#print(warped_img)
		gray_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
		sobel_x = cv2.Sobel(gray_img,cv2.CV_64F, 1, 0, ksize=kernel_size)
		sobel_y = cv2.Sobel(gray_img,cv2.CV_64F, 0, 1, ksize=kernel_size)

		abs_sobel_x = np.absolute(sobel_x)
		abs_sobel_y = np.absolute(sobel_y)

		gradient= np.sqrt(sobel_x**2 + sobel_y**2)
		arc_tan = np.arctan2(abs_sobel_y,abs_sobel_x)

		v=abs_sobel_x

		if(sobel_type=='x'):
			v=abs_sobel_x
		elif(sobel_type=='y'):
			v= abs_sobel_y
		elif(sobel_type=='xy'):
			v = gradient
		else:
			v = arc_tan

		img = np.uint8((v * 255)/np.max(v))
		output = np.zeros_like(img)
		output[(img > th[0]) & (img < th[1])]=1
		return output

	def histogram(self,img):
		## 이미지의 넓이(테스트 영상의 경우 1280 사이즈), 행의 색상 합을 구한 히스토그램을 만들어냄
		## 관심영역만 쓸것이므로 //2 
		# img.shape = width, height
		return np.sum(img[img.shape[0]//2:,:], axis=0)

	@property
	def src(self):
		return self.__src

	@property
	def dst(self):
		return self.__dst
	
	
