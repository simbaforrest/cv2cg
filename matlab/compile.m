mex -O cvsurf.cpp;
!g++ -O3 -funroll-loops -o cvfundmat cvfundmat.cpp -I../include -I/usr/local/include -I/usr/local/include/opencv  -L/usr/local/lib64 -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann -lm
