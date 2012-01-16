/*
* ImageManip.h
* 
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef IMAGE_MANIP_H
#define IMAGE_MANIP_H

// QT libraries
#include <QString>

class ImageManip
{
public:
	static bool ResizeImage(QString src, QString dst, int percent);
	static bool ResizeImageWindows(QString src, QString dst, int percent);
	static bool ResizeImageLinux(QString src, QString dst, int percent);

	static bool TileImage(QString src, QString dst, int size);
	static bool TileImageWindows(QString src, QString dst, int size);
	static bool TileImageLinux(QString src, QString dst, int size);

	static bool GetDimensions(QString path, int& width, int& height);
	static bool GetDimensionsWindows(QString path, int& width, int& height);
	static bool GetDimensionsLinux(QString path, int& width, int& height);

private:
	static bool knownPlatform;
	static bool isWindows;
	static bool isLinux;
};

#endif // IMAGE_MANIP_H
