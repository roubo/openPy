#!/usr/bin/python2.7
#encoding:utf-8
import operator
from numpy import *
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from os import listdir
"""
kNN应用于手写数字图像的识别
"""
class kNNforImage:
    def __init__(self):
        print "Init the class kNNforImage.\n"
    def _classify0(self, inX, dataSet, labels, k):
        """
        k-近邻算法分类:classify0(self, inX, dataSet, labels, k)
        inX    ：    输入量
        dataSet:    训练样本集
        labels :    训练样本标签
        Ｋ     ：    近邻数目
        """
        dataSetSize = dataSet.shape[0] #数据集合的数量
        diffMat = tile(inX, (dataSetSize, 1))-dataSet  #扩充inX数组为和dataSet大小一样，然后元素作差
        sqDiffMat = diffMat**2 #求平方
        sqDistances = sqDiffMat.sum(axis=1) #列求和
        distance = sqDistances**0.5 #求开根
        #print distance
        sortedDistInd = distance.argsort()#升序排序
        #print sortedDistInd
        classCount={}#创建字典
        for i in range(k):
            voteIlabel = labels[sortedDistInd[i]]
            classCount[voteIlabel] = classCount.get(voteIlabel,0) + 1
        #print classCount
        #将字典排序之前，需要将字典转换为列表或者迭代对象，然后可以采用key方式索引
        sortedClasscount = sorted(classCount.iteritems(),key=lambda x:x[1], reverse = True)
        #print sortedClasscount
        return sortedClasscount[0][0]
    def _img2vector(self,filename):
        """
        测试（训练）数据准备，将现有的图像数据格式化为可作为分类器输入的结构
        将每个图像数据文件当作一个样本
        filename :   图像数据文件
        返回一个样本数据
        """
        returnVect = zeros((1,1024))#一个样本，相当于有1024个特征值
        filep = open(filename)
        for i in range(32): #大小已知 32行
            lineStr = filep.readline()
            for j in range(32):#32列
                returnVect[0,32*i+j] = int(lineStr[j])
        return returnVect
    def _show3D(self):
        """
        尝试将手写数字图像数据用3Ｄ形式体现 散点图的形势
        Matrix:  单个样本体现出来的矩阵32*32
        """
        fig = plt.figure()
        ax = Axes3D(fig)
        #构造三个长度一致的列表32×32
        Y = zeros((1,32*32))
        e = ones((1,32))
        for i in range(32):
            Y[0,32*i:32*i+32]=e*i
        y=len(Y)
        print y 
        e1 = arange(0,32,1)
        X = tile(e1,32) 
        x=len(X)
        print x
        Z = zeros((1,32*32))
        Z = self._img2vector('../trainingDigits/2_2.txt')
        ax.scatter3D(X, Y, Z, 'z')
        plt.show()
        
    def _hwClassTest(self):
        """
        手写数字识别分类测试
        从两个目录下分别获取训练数据和测试数据
        得出错误率
        """
        hwLabels = []#准备训练样本的列表
        trainFileList = listdir('../trainingDigits')#获取目录下的文件名列表
        #通过文件名来解析出来有用的训练数据
        m = len(trainFileList)
        trainMatrix = zeros((m,1024))#构造训练数据矩阵
        for i in range(m):#总训练样本数ｍ
            filenameStr = trainFileList[i]
            fileStr = filenameStr.split('.')[0]#使用.来分割为列表，并取有用部分
            classNumStr = int(fileStr.split('_')[0])#同上。获得类别标签
            hwLabels.append(classNumStr)
            trainMatrix[i,:] = self._img2vector('../trainingDigits/%s' % filenameStr)#具体数据
        #通过文件名来解析出测试数据
        testFileList = listdir('../testDigits')
        errorcount = 0.0
        mtest = len(testFileList)
        for i in range(mtest):
            filenameStr = testFileList[i]
            fileStr = filenameStr.split('.')[0]
            classNumStr = int(fileStr.split('_')[0])#正确答案
            testVetor = self._img2vector('../testDigits/%s' % filenameStr)
            classfiyresult = self._classify0(testVetor, trainMatrix, hwLabels, 3)
            print "The classifier get:%d ,the real result is: %d\n" % (classfiyresult, classNumStr)
            if(classfiyresult != classNumStr):errorcount+=1.0
        print "The total error is :%d\n" % errorcount
        print "The error rate is :%f\n" %(errorcount/float(mtest)) 
    
    def run(self):
        """
        类测试
        """
        self._hwClassTest() 
        self._show3D()

def main():
    knnforimage = kNNforImage()
    knnforimage.run()

if __name__ == "__main__":
    main()