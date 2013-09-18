#!/usr/bin/python2.7
#encoding:utf-8
import operator
from numpy import *
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
"""
kNN分类作用于“约会”网站的测试
"""
class kNNforDate:
    def __init__(self):
        print "init the class kNNforDate.\n"
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
    def _file2matrix(self,filename='datingTestSet.txt'):
        """
        将来自文件的数据转载到列表中，即为算法构建合理的格式化数据结构。
        filename :    数据文件名
        返回测试数据矩阵和标签列表
        """
        filep = open(filename)
        arrayOLines = filep.readlines()#按行读取
        #print arrayOLines
        numberOfLines = len(arrayOLines)#获取行数(这里测量的是元素个数)
        returnMatrix = zeros((numberOfLines,3))#构造测试数据矩阵，3个特征值
        classLabelVector = []#构造测试标签列表结构
        index = 0
        for line in arrayOLines:
            line = line.strip()#取出一行，去除多余的空格和回车
            listFromline = line.split('\t')#根据制表符分割到一个列表中
            returnMatrix[index,:] = listFromline[0:3]#前三个元素给矩阵
            classLabelVector.append(int(listFromline[-1]))#最后一个元素给标签列表
            index+=1
        return returnMatrix, classLabelVector 
    def _show2D(self,Matrix,Label):
        """
        绘制数据的2Ｄ表现图表
        使用pyplot模块，该模块的常用类关系：Figure->Axes->(Line2D,Text,)
        Matrix:   数据矩阵
        Label：    数据矩阵对应的类别列表
        """
        #print Matrix
        #print Label
        fig = plt.figure()#创建一个figure
        ax1 = fig.add_subplot(2,1,1)#创建一个axes，即子图，并指向操作的子图
        ax2 = fig.add_subplot(2,1,2)
        ax1.scatter(Matrix[:,0],Matrix[:,1],15.0*array(Label),15.0*array(Label))
        ax2.scatter(Matrix[:,1],Matrix[:,2],15.0*array(Label),15.0*array(Label))
        ax1.axis([-5000,100000,-2,25])
        ax2.axis([-1,25,-0.2,2.0])
        plt.show()
    def _show3D(self,Matrix,Label):
        """
        绘制数据的3Ｄ表现图表
        Matrix:   数据矩阵
        Label：    数据矩阵对应的类别列表
        """
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter3D(Matrix[:,0], Matrix[:,1], Matrix[:,2], 'z',15.0*array(Label),15.0*array(Label))
        plt.xlabel('Frequent Flyier Miles ')
        plt.ylabel('Time of Playing Video Games')
        plt.show()
        
    def _autoNorm(self,dataSet):
        """
        归一化所有数据
        dataSet:      数据矩阵
        """
        minVals = dataSet.min(0)#每列中最小值组成一个新的列表
        maxVals = dataSet.max(0)#每列中最大值组成一个新的列表
        ranges = maxVals-minVals#每列（一个特征项）中的浮动范围
        normDataSet = zeros(shape(dataSet))#构造新的数据矩阵结构
        m = dataSet.shape[0]#数据矩阵的数量
        #print m
        normDataSet = dataSet - tile(minVals, (m,1))
        normDataSet = normDataSet/tile(ranges, (m,1))
        return normDataSet, ranges, minVals
    def _datingClassTest(self):   
        """
        使用该类数据，测试分类的准确性
        """
        hoRatio = 0.1 #准确性验证样本数量占总量的比例
        dataset,datalabel = self._file2matrix('datingTestSet.txt')
        normdata,range,minval = self._autoNorm(dataset)
        m = normdata.shape[0]
        n = int(m*hoRatio)
        #print n 
        errorcount = 0.0
        for i in arange(n): 
            labelResult = self._classify0(normdata[i,:], normdata[n:m,:], datalabel[n:m], 3)
            print "The classifier get:%d, the real one is:%d\n" % (labelResult,datalabel[i])
            if (labelResult != datalabel[i]): errorcount+=1.0
        print "The fault rate is: %f\n" % (errorcount/float(n))
        
    def _classifyperson(self):
        resultList = ['Not at all', 'In samll doses', 'In large doses']
        #用户输入测试数据
        percentTats = float(raw_input("PercentTat of time spent playing video games:"))
        ffMiles = float(raw_input("Frequent filer miles earned per year:"))
        iceCream = float(raw_input("Liters of ice cream per year:"))
        testset = array([ffMiles,percentTats,iceCream])#需预测的数据
        dataset,label = self._file2matrix('datingTestSet.txt')
        normMat, rang, minVals = self._autoNorm(dataset)
        classfiyresult = self._classify0((testset-minVals)/rang, normMat, label, 3)
        print "You will like this person: ", resultList[classfiyresult-1]
                   
    def run(self):
        """
        方便交互界面测试该class
        """
        #self.__init__()
        #m,c = self._file2matrix()
        #print m
        #print c 
        #self._show2D(m, c)
        #self._show3D(m, c)
        #m1,rang,min=self._autoNorm(m)
        #print m1
        #self._datingClassTest()
        self._classifyperson()
        
def main():
    """
    测试函数
    """    
    knnfordate = kNNforDate()
    knnfordate.run()
    
if __name__ == "__main__":
    main()  