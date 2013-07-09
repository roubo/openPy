/******************************
 * 语音识别测试单例
 * 感知识别接口层 FRL
 * 静态音频识别
 * roubo_dai@san412.in
 * San412
 * ****************************/
#include <pocketsphinx.h>
//#define MODELDIR "/usr/local/share/pocketsphinx/model"
int main(int argc,char **argv)
{
	ps_decoder_t      *ps;//解码器
	cmd_ln_t          *config;//解码器配置
	FILE 		  *fh;//音频文件句柄
	int 		  rv;
	char const        *hyp,*uttid;
	int16 		  buf[512];
	int32 		  score;
	//初始化解码器配置
	//-hmm      声学模型
	//-lm       语言模型
	//-dict     字典文件
	//MODELDIR  模型所在目录 宏定义
	config = cmd_ln_init(NULL, ps_args(),TRUE, 
			"-hmm", "/usr/local/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k",
			"-lm",  "/usr/local/share/pocketsphinx/model/lm/en/turtle.DMP",
			"-dict","/usr/local/share/pocketsphinx/model/lm/en/turtle.dic",
			NULL);
	if(config == NULL)
		return 1;
	
	//初始化解码器
	ps = ps_init(config);
	if(ps == NULL)
		return 1;
	//打开被识别音频文件
	fh = fopen("../../goforward.raw","rb");
	if(fh == NULL)
	{
		printf("Failed to open goforward.raw\n");
		return 1;
	} 
	//使用ps_decode_raw直接开始解码
	rv = ps_decode_raw(ps, fh, NULL, -1);
	if(rv < 0)
	{
		printf("Failed to decode the raw.\n");
		return 1;
	}
	//提取结果
	hyp = ps_get_hyp(ps, &score, &uttid);
	if(hyp == NULL)
	{
		printf("Cann`t recognition.\n");
		return 1;
	}
	printf("recognized:%s\n",hyp);
	fclose(fh);
	ps_free(ps);
	exit(0);

}


