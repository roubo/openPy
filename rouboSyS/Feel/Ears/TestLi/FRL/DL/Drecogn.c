/******************************
 * 语音识别测试单例
 * 感知识别接口层 FRL
 * 动态音频识别
 * roubo_dai@san412.in
 * San412
 * ****************************/
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>

#include <pocketsphinx.h>
//使用sphinxbase的ＡＬＳＡ接口，对声卡的控制
#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/cont_ad.h>

ps_decoder_t 	*ps;//解码器
cmd_ln_t        *config;//解码器配置 

/******************************
 * IO阻塞超时设置
 *****************************/
static void sleep_msec(int32 ms)
{
	struct timeval   tm;
	tm.tv_sec  = 0;
	tm.tv_usec = ms*1000;
	select(0,NULL,NULL,NULL,&tm);
}

/*******************************
 * 实时语音识别
 *******************************/
static void recognize_from_mic()
{
	ad_rec_t      *ad;  //音频设备对象
	cont_ad_t     *cont;//连续监听模式配置对象
	char const    *hyp,*uttid;
	int16         adbuf[4096];
	char          word[256];
	int32         k,ts,rem;
	
	//打开音频设备
	if((ad = ad_open_dev(cmd_ln_str_r(config,"-adcdev"),(int)cmd_ln_float32_r(config, "-samprate"))) == NULL)
		E_FATAL("Failed to open audio device\n");
	//初始化连续监听模式
	if((cont = cont_ad_init(ad, ad_read)) == NULL)
		E_FATAL("Failed to init continuous listening module.\n");
	//开始录制
	if(ad_start_rec(ad) < 0)
		E_FATAL("Failed to start recording.\n");
	//按照监听模式校正音频设备
	if(cont_ad_calib(cont) <0)
		E_FATAL("Failed to calibrate voice dev.\n");

	//循环识别语音输入指令
	for(;;)
	{
		printf("ready...\n");
		fflush(stdout);
		fflush(stderr);
		
		//等待语音指令的开始
		while((k = cont_ad_read(cont, adbuf, 4096)) == 0)
			sleep_msec(100);
		if(k < 0)
			E_FATAL("Failed to read audio.\n");
		/*开始分段语音识别*/
		if(ps_start_utt(ps, NULL) <0)
			E_FATAL("Failed to start utterance.\n");
		//处理上面的片段
		ps_process_raw(ps, adbuf, k, FALSE, FALSE);
		printf("listening,,,\n");
		fflush(stdout);
		//提取时间戳
		ts = cont->read_ts;
		//循环处理剩下的片段
		for(;;)
		{
			//无阻塞ＩＯ
			if((k = cont_ad_read(cont, adbuf, 4096)) <0)
				E_FATAL("Failed to read audio.\n");
			//如果没数据，查看时间戳，如果静音时间大于一定时间则结束该语音指令
			if(k == 0)
			{
				if((cont->read_ts - ts)>DEFAULT_SAMPLES_PER_SEC)
					break;
			}
			//否则，更新时间戳
			else
			{
				ts = cont->read_ts;
			}

			//处理上面的片段
			rem = ps_process_raw(ps, adbuf, k, FALSE, FALSE);
			if((rem == 0)&&(k == 0))
				sleep_msec(20);
		}
		/*结束分段语音处理*/

		//关闭监听，重设连续监听模式
		ad_stop_rec(ad);
		while(ad_read(ad, adbuf, 4096) >=0);
		cont_ad_reset(cont);
		printf("stop listening...\n");
		fflush(stdout);
		ps_end_utt(ps);

		//提取识别信息
		hyp = ps_get_hyp(ps, NULL, &uttid);
		printf("%s:%s.\n",uttid, hyp);
		fflush(stdout);
		//测试goodbye
		if(hyp)
		{
			sscanf(hyp,"%s",word);
			if(strcmp(word, "REST") == 0)
			{
				printf("Yes, rest mode.\n");
				fflush(stdout);
				//break;
				exit(2);
			}
			if(strcmp(word, "WORK") == 0)
			{
				printf("Yes,work mode.\n");
				fflush(stdout);
				exit(3);
			}
		}
		//重启音频设备
		if(ad_start_rec(ad)<0)
			E_FATAL("Failed to resume recording.\n");
	
	} 
	cont_ad_close(cont);
	ad_close(ad);
}

int main(int argc,char **argv)
{

	
	//初始化解码器配置
	//-hmm      声学模型
	//-lm       语言模型
	//-dict     字典文件
	//MODELDIR  模型所在目录 宏定义
	config = cmd_ln_init(NULL, ps_args(),TRUE,
			//模型en
			/*"-hmm", "/home/roubo/CSRsphinx/en/hhmbraodcastnews_16K/",
                          "-lm",  "/home/roubo/CSRsphinx/en/WSJ20K_trigram_lm/tcb20onp.Z.DMP",
                          "-dict","/home/roubo/CSRsphinx/en/cmudict.hub4.06d.dict",
                       NULL);*/
			//模型en
			/* "-hmm", "/home/roubo/CSRsphinx/en/hhmhub4opensrc/",
                         "-lm",  "/home/roubo/CSRsphinx/en/HUB4_trigram_lm/language_model.arpaformat.DMP",
                         "-dict","/home/roubo/CSRsphinx/en/cmudict.hub4.06d.dict",
                       NULL);*/
			//模型zh
			/*"-hmm", "/home/roubo/CSRsphinx/zh/zh_broadcastnews_ptm256_8000/",
			"-lm",  "/home/roubo/CSRsphinx/zh/zh_broadcastnews_64000_utf8.DMP",
			"-dict","/home/roubo/CSRsphinx/zh/zh_broadcastnews_utf8.dic",
			NULL);*/
			//自带模型en
			/*"-hmm", "/usr/local/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k",
			"-lm",  "/usr/local/share/pocketsphinx/model/lm/en/turtle.DMP",
			"-dict","/usr/local/share/pocketsphinx/model/lm/en/turtle.dic",
			NULL);i*/
			//自带模型zh
			/*"-hmm", "/usr/local/share/pocketsphinx/model/hmm/zh/tdt_sc_8k/",
                          "-lm",  "/home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/AC/EarTrain/data/zh/arctic.DMP",
                         "-dict","/usr/local/share/pocketsphinx/model/lm/zh_CN/mandarin_notone.dic",
                        NULL);*/
			"-hmm", "/usr/local/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k",
                         "-lm",  "/home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/AC/EarTrain/data/en/testli.lm",
                         "-dict","/home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/AC/EarTrain/data/en/testli.dic",
                         NULL);
	if(config == NULL)
		return 1;
	
	//初始化解码器
	ps = ps_init(config);
	if(ps == NULL)
		return 1;
	recognize_from_mic();
	ps_free(ps);
	exit(0);

}


