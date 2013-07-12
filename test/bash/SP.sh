#!/bin/bash
for((i=0;i<4;i++)); do
{
	sleep 2
	echo 1 >> aa %% echo "done"
}
done
cat aa | wc -l
rm aa
