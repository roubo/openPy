#!/usr/bin/python2.7
#encoding:utf-8
from math import log
import operator
"""
决策树的构造，分类算法测试
"""
class Dtree:
    def __init__(self):
        print "Init the class Dtree.\n"
    
    def _createDataSet(self):
        """
        构建测试样本数据矩阵
        """
        dataSet = [[1,1,'yes'],[1,1,'yes'],[1,0,'no'],[0,1,'no'],[0,1,'no']]
        labels = ['No surfacing', 'flippers']
        return dataSet,labels
    
    def _calacShannonEnt(self,dataSet):
        """
        计算数据矩阵的香农熵
        即为数据矩阵中所有分类的可能值的信息期望值
        dataSet :   数据矩阵，最后一列为分类标签
        """
        labelCounts = {}#创建分类字典
        n = len(dataSet)#样本数量
        for fectVec in dataSet:
            currentlabel = fectVec[-1]#提取该样本的分类标签
            if currentlabel not in labelCounts.keys():#检测该标签是否已存在
                labelCounts[currentlabel] = 0#字典中新建一个key，并赋值为0
            labelCounts[currentlabel] += 1#计算重复次数
        shannonEnt = 0.0
        for key in labelCounts:#用key来做循环界限
            prob = float(labelCounts[key])/n #计算该分类的该类
            shannonEnt -= prob * log(prob,2)#按照定义计算熵
        return shannonEnt
    
    def _splitDataSet(self,dataSet,axis,value):
        """
        根据给定的特征来分割数据矩阵,作为数分叉的体现，
        针对同一个特征，特征值的不同就需要多次分割，才是完成针对某一特征的分割
        dataSet :    数据矩阵，该列表内容不允许修改
        axis:        给定特征在dataSet中的位置
        value:       给定特征满足的值
        返回被划分过的矩阵
        """
        retDataSet = []
        for featVec in dataSet:
            if featVec[axis] == value:#满足分割条件
                #将该特征元素从中去掉，表示已经经过此类划分
                reducefeatVec = featVec[:axis]
                reducefeatVec.extend(featVec[axis+1:])#extend是扩充列表本身
                retDataSet.append(reducefeatVec)#append是扩充矩阵
        return retDataSet
    
    def _chooseBestFeatureToSplit(self,dataSet):
        """
        根据所有的特征，来分割矩阵，并计算每种分割后的信息增益，得到当前最好的分割方式
        dataSet :   需要被分割的矩阵
        返回：  最好的特征元素在数据矩阵中的索引号
        """
        nFeature = len(dataSet[0])-1 #当前数据矩阵内的特征数量
        baseEntropy = self._calacShannonEnt(dataSet)#当前数据矩阵的熵
        bestInfoGain = 0.0
        bestFeature = -1
        
        for i in range(nFeature):
            #提取i号索引下的所有特征值（无重复），作为分割时的参考
            featList = [example[i] for example in dataSet]#使用列表推导，循环提取特征值（有重复）
            uniquefeatList = set(featList)#使用集合set函数将重复值去掉
            newEntropy = 0.0
            for value in uniquefeatList:
                subDataSet = self._splitDataSet(dataSet, i, value)#获取以当前特征值为参考的分割出来的数据矩阵
                #计算该数据矩阵的熵，并加入到以该特征（而非特征值）进行分类时的总的熵中
                prob = len(subDataSet)/float(len(dataSet))
                newEntropy += prob * self._calacShannonEnt(subDataSet)
            infoGain = baseEntropy - newEntropy
            if(infoGain > bestInfoGain):#交换最大信息增益和特征索引号
                bestInfoGain = infoGain
                bestFeature = i 
        return bestFeature
    
    def _majorityCnt(self,classList):
        """
        当数据矩阵上所有的特征都遍历过以后，节点上的样本类标签依然有不同的，此时使用该函数确定下来
        classList :  需要被确定的节点的类列表
        """
        #类似kNN分类算法中的投票表代码，对字典进行排序
        classCount={}#字典
        for vote in classList:
            if vote not in classCount.keys():classCount[vote] = 0
            classCount += 1
        sortedClassCount = sorted(classCount.iteritems(),key=operator.itemgetter(1),reverse = True)
        return sortedClassCount[0][0]
    
    def _createTree(self,dataSet,labels):
        """
        创建决策树,完成树的生长
        dataSet :    数据矩阵，最后一列需要是分类标签
        labels:      特征的名称列表，只是为了形象化特征
        """
        classList = [example[-1] for example in dataSet]# 推导获取分类标签
        if classList.count(classList[0]) == len(classList):# 统计当前数据矩阵中的某一分类标签的数量，若为整长，则分类结束
            return classList[0]
        if len(dataSet) == 1:#如果消耗完所有特征，则结束
            return self._majorityCnt(classList)
        bestFeat = self._chooseBestFeatureToSplit(dataSet)#选择当前最合适的特征
        bestFeatLabel = labels[bestFeat]#该特征的名称
        #print bestFeatLabel
        mytree = {bestFeatLabel:{}}#嵌套字典，在迭代时可以自动嵌套
        del(labels[bestFeat])#每次迭代，特征名称也就少一个
        featValues = [example[bestFeat] for example in dataSet]
        uniqueFeatValues = set(featValues)
        for value in uniqueFeatValues:
            subLabels = labels[:]
            mytree[bestFeatLabel][value]=self._createTree(self._splitDataSet(dataSet, bestFeat, value), subLabels)
        return mytree
    
    def run(self):
        myData,labels = self._createDataSet()
        print myData
        print labels
        mytree = self._createTree(myData, labels)
        print mytree
        
def main():
    dtree = Dtree()
    dtree.run()
    
if __name__ == "__main__":
    main()
        
        
            
            