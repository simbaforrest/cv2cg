clear
clear
rm -f ./test

SOURCE=testesm.cpp

INCLUDE_DIR="-I/usr/local/include -I/usr/local/include/opencv -I/usr/local/include/eigen3 -I."
LIB_DIR="-L/usr/local/lib64 -L/usr/local/lib"

OPENCV_LIBS="-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann"

OSG_LIBS="-losg -losgDB -losgFX -losgGA -losgParticle -losgSim -losgText -losgUtil -losgTerrain -losgManipulator -losgViewer -losgWidget -losgShadow -losgAnimation -losgVolume -lOpenThreads"

OTHER_LIBS="esm/ESMlib.a -lm"

##check preprocessed result
#g++ -E test.i ${SOURCE} ${INCLUDE_DIR} ${LIB_DIR} ${OPENCV_LIBS} ${OSG_LIBS} ${OTHER_LIBS} >>test.i

##debug
g++ -g -o test ${SOURCE} ${INCLUDE_DIR} ${LIB_DIR} ${OPENCV_LIBS} ${OSG_LIBS} ${OTHER_LIBS}

##release
#g++ -O3 -funroll-loops -o test ${SOURCE} ${INCLUDE_DIR} ${LIB_DIR} ${OPENCV_LIBS} ${OSG_LIBS} ${OTHER_LIBS}
