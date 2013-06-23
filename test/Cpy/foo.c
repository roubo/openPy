#include <Python.h>
#include <stdio.h>

//ģ���ڷ����ľ���ʵ��
//ֻ�ڲ���Ч��������ӿ�
//����ȡ������Ӱ�����
static PyObject* foo_bar(PyObject* self, PyObject* args)
{
	Py_RETURN_NONE;
}

//ģ���ڷ����ľ���ʵ��
//��python����3��������3���ɱ��Ĭ�ϲ���
//��ӡ����
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

//ģ���ڷ����ľ���ʵ��
//��python���չؼ��ֲ���
//��ӡ��������python���ز���
static PyObject* foo_bar3(PyObject* self, PyObject* args, PyObject* kw)
{
	static char* kwlist[] = {"i","d","s",NULL};//�ؼ����ֵ�

	int     iNum=0;
	double  fNum=3.0f;
	char*   str="san412";
	if(!PyArg_ParseTupleAndKeywords(args,kw,"i|ds",kwlist, &iNum, &fNum, &str))
	{
		printf("error.\n");
		return NULL;
	}
	printf("Get data: %d,%f,%s\n",iNum,fNum,str);
	return Py_BuildValue("i",iNum+1);//���ص�python
}
//ģ���ڵķ����б�
//��������bar
static PyMethodDef foo_methods[]={
	{"bar", (PyCFunction)foo_bar, METH_NOARGS, NULL},
	{"bar2",(PyCFunction)foo_bar2,METH_VARARGS, NULL},
	{"bar3",(PyCFunction)foo_bar3,METH_VARARGS|METH_KEYWORDS,NULL},
	{ NULL, NULL, 0, NULL}
};



//�ú�����python���ظ�ģ���ʱ�򱻵���
//ģ������foo
PyMODINIT_FUNC initfoo(){
	Py_InitModule3("foo", foo_methods, "C Ext Module.");
}
