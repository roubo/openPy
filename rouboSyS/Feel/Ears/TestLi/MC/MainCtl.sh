#!/bin/bash
##############################
# ����ʶ����Ե���
# ��Э�����ƽű�
# roubo_dai@san412.in
# roubo
###############################
sleep 10
bash -x /home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/FOS/order_Drecogn.sh
com=$(cat /home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/FOS/order_Drecogn_commumication)
if [ $com == 2 ];then
	echo "Rest mode!"
	bash -x /home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/FOS/order_Restmode.sh
elif [ $com == 3 ];then
	echo "Work mode!"
	bash -x /home/roubo/openPy/rouboSyS/Feel/Ears/TestLi/FOS/order_Workmode.sh
else
	echo "Something Wrong!!"
fi

