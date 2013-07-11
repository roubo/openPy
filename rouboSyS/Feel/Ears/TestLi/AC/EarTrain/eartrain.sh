#!/bin/bash
PATH=/bin:/sbin/:/usr/bin/:/usr/sbin/:/usr/local/bin/:/usr/local/sbin/
export PATH

echo "rouboSyS: Ears train script."
########################################
# 语音识别测试单例
# 辅助控制系统
# 语言模型和被适应优化的声学模型训练脚本
# roubo_dai@san412.in
# San412
########################################

# 使用语言模型训练工具集CMUCLMTK #

# 训练文本和一些过渡文件
TrainTxt="./data/zh/arctic.txt"
Tmpvocabulary="./data/zh/arctic.tmp.vocab"
Tmpidngram="./data/zh/arctic.tmp.idngram"
Tmparpa="./data/zh/arctic.tmp.arpa"
Tmpdmp="./data/zh/arctic.DMP"

# 生成词汇表vocabulary文件
text2wfreq < $TrainTxt | wfreq2vocab > $Tmpvocabulary

# 生成arpa格式的语言模型
text2idngram -vocab $Tmpvocabulary -idngram $Tmpidngram < $TrainTxt
idngram2lm -vocab_type 0 -idngram $Tmpidngram -vocab $Tmpvocabulary -arpa $Tmparpa

# 生成DMP类型的语言模型
sphinx_lm_convert -i $Tmparpa -o $Tmpdmp
