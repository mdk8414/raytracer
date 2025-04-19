#!/bin/bash
IMAGE=$1
TEST_PATH="../../raytracer-files"
make run file=./raytracer-files/ray-$IMAGE.txt
echo "mkdir my_tests"
mkdir my_tests
echo "cd my_tests"
cd my_tests
echo "mkdir $IMAGE"
mkdir $IMAGE
echo "${pwd}"
pwd
echo "mv -f ../$IMAGE.png $IMAGE"
#mv -f ../$IMAGE.png $IMAGE
cp -f ../$IMAGE.png $IMAGE
cd $IMAGE
compare -fuzz 2% $IMAGE.png $TEST_PATH/ray-$IMAGE.png ae_$IMAGE.png
composite $IMAGE.png $TEST_PATH/ray-$IMAGE.png -alpha off -compose difference rawdiff_$IMAGE.png
convert rawdiff_$IMAGE.png -level 0%,8% diff_$IMAGE.png
convert +append $TEST_PATH/ray-$IMAGE.png $IMAGE.png ae_$IMAGE.png rawdiff_$IMAGE.png diff_$IMAGE.png comp_$IMAGE.png


cp -f comp_$IMAGE.png ../../comparisons