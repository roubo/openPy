#!/usr/bin/python2.7
#encoding:utf-8
from numpy import *
import operator
import sys
"""
  k-近邻算法，测试代码
"""

class kNN:
    def __init__(self):
        print " init class kNN\n"
    def _createDataSet(self):
        """
        创建演示程序的数据集和标签
        """
        self.group = array([[1.0,1.1],[1.0,1.0],[0,0],[0,0.1]])
        self.labels = ['A','A','B','B']
        return self.group, self.labels
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
        print distance
        sortedDistInd = distance.argsort()#升序排序
        print sortedDistInd
        classCount={}
        for i in range(k):
            voteIlabel = labels[sortedDistInd[i]]
            classCount[voteIlabel] = classCount.get(voteIlabel,0) + 1
        sortedClasscount = sorted(classCount.iteritems(),key=operator.itemgetter(1), reverse = True)
        return sortedClasscount[0][0]
        
    
def main():
        """
        测试主函数
        """
        knn=kNN()
        g,l=knn._createDataSet()
        print g
        print "\n"
        print l
        print "\n"
        inX = array([0,0])
        retclass = knn._classify0(inX, g, l, 3)
        print "class:%s\n",%retclass

if __name__ =='__main__':
    main()
