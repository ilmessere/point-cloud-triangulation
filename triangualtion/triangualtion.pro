TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#LIBS += -L"/usr/include/pcl-1.7"

SOURCES += main.cpp \
    pointcloudtriangulation.cpp \
    pclvoxelgridfilter.cpp \
    pclstatisticaloutlierremoval.cpp
    #pclsurfacesmoothing.cpp

HEADERS += \
    pointcloudtriangulation.h \
    pclvoxelgridfilter.h \
    pclstatisticaloutlierremoval.h \
    #pclsurfacesmoothing.h
    pcfilter.h

unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lpcl_apps

INCLUDEPATH += $$PWD/../../../../../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../../../../../usr/include/pcl-1.7

unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lpcl_common

INCLUDEPATH += $$PWD/../../../../../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../../../../../usr/include/pcl-1.7

unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lpcl_io

INCLUDEPATH += $$PWD/../../../../../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../../../../../usr/include/pcl-1.7

unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lpcl_search -lpcl_features

INCLUDEPATH += $$PWD/../../../../../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../../../../../usr/include/pcl-1.7

unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lpcl_kdtree -lpcl_surface -lpcl_filters

INCLUDEPATH += $$PWD/../../../../../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../../../../../usr/include/pcl-1.7

INCLUDEPATH += $$PWD/../../../../../../../usr/include/eigen3/


#unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lboost_timer

INCLUDEPATH += $$PWD/../../../../../../../usr/include/boost
DEPENDPATH += $$PWD/../../../../../../../usr/include/boost


unix:!macx: LIBS += -L$$PWD/../../../../../../../usr/lib/ -lboost_system

INCLUDEPATH += $$PWD/../../../../../../../usr/include/boost
DEPENDPATH += $$PWD/../../../../../../../usr/include/boost
