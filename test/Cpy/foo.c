#include <Python.h>
#include <stdio.h>

//模块内方法的具体实现
//只内部有效，不对外接口
//随意取名，不影响调用
static PyObject* foo_bar(PyObject* self, PyObject* args)
{
	Py_RETURN_NONE;
}

//模块内方法的具体实现
//从python接收3个参数，3个可变的默认参数
//打印参数
static PyObject* foo_bar2(PyObject* self, PyObject* args)
{
	int     iNum;
	double  fNum;
	char*   str;

	int     iNum2=412;
	double  fNum2=3.0f;
	char*   str2="san412";
	if(!PyArg_ParseTuple(args, "ids|ids", &iNum, &fNum, &str, &iNum2, &fNum2, &str2))
	{
		return NULL;
	}
	printf("Get data: %d,%f,%s\n",iNum,fNum,str);
	printf("default data: %d,%f,%s\n",iNum2,fNum2,str2);
	Py_RETURN_NONE;
}

//模块内方法的具体实现
//从python接收关键字参数
//打印参数，向python返回参数
static PyObject* foo_bar3(PyObject* self, PyObject* args, PyObject* kw)
{
	static char* kwlist[] = {"i","d","s",NULL};//关键字字典

	int     iNum=0;
	double  fNum=3.0f;
	char*   str="san412";
	if(!PyArg_ParseTupleAndKeywords(args,kw,"i|ds",kwlist, &iNum, &fNum, &str))
	{
		printf("error.\n");
		return NULL;
	}
	printf("Get data: %d,%f,%s\n",iNum,fNum,str);
	return Py_BuildValue("i",iNum+1);//返回到python
}
//模块内的方法列表
//方法名：bar
static PyMethodDef foo_methods[]={
	{"bar", (PyCFunction)foo_bar, METH_NOARGS, NULL},
	{"bar2",(PyCFunction)foo_bar2,METH_VARARGS, NULL},
	{"bar3",(PyCFunction)foo_bar3,METH_VARARGS|METH_KEYWORDS,NULL},
	{ NULL, NULL, 0, NULL}
};



//该函数在python加载该模块的时候被调用
//模块名：foo
PyMODINIT_FUNC initfoo(){
	Py_InitModule3("foo", foo_methods, "C Ext Module.");
}
