/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */
#include "Utils/GTexture.hpp"
#include "Utils/Misc.hpp"
#include <cstdlib>
#include <GL/glu.h>
#include <png.h>

namespace Antipatrea
{
    GTexture::GTexture(void)
    {
	m_id    = 0;
	m_image = NULL;
	m_fname = NULL;
	m_sizeX = 0;
	m_sizeY = 0;
    }
    
    GTexture::~GTexture(void)
    {
	if(m_fname)
	    free(m_fname);
	if(m_image)
	{
	    free(m_image);
	    glDeleteTextures(1, &m_id);
	}	    
    }
        
    void GTexture::ReadPPM(FILE *in)
    {
	int i,d;
	char head[70]; 
	char *ignore;
	size_t   iignore;
	
	ignore = fgets(head, 70, in);
	if(strncmp(head, "P6", 2)) 
	{
	    fprintf(stderr, "not a raw PPM file\n");
	    return;
	}
	
	i = 0;
	while(i < 3) 
	{
	    ignore = fgets(head, 70, in);
	    if (head[0] == '#')     /* skip comments. */
		continue;
	    if (i == 0)
		i += sscanf(head, "%d %d %d", &m_sizeX, &m_sizeY, &d);
	    else if (i == 1)
		i += sscanf(head, "%d %d", &m_sizeY, &d);
	    else if (i == 2)
		i += sscanf(head, "%d", &d);
	}
	
	m_image = (unsigned char*)malloc(sizeof(unsigned char) * m_sizeX * m_sizeY * 3);
	iignore = fread(m_image, sizeof(unsigned char), m_sizeX * m_sizeY * 3, in);
    }
    
    void GTexture::ReadBMP(FILE * in)
    {
	unsigned long sizeX;
	unsigned long sizeY;
	unsigned long size;                 // size of the image in bytes.
	unsigned long i;                    // standard counter.
	unsigned short int planes;          // number of planes in image (must be 1) 
	unsigned short int bpp;             // number of bits per pixel (must be 24)
	char temp;                          // used to convert bgr to rgb color.
	 
	// Skip to bmp header
	fseek(in, 18, SEEK_CUR);
	
	if ((i = fread(&sizeX, 4, 1, in)) != 1) 
	    return;
	if((i = fread(&sizeY, 4, 1, in)) != 1) 
	    return;
	size = sizeX * sizeY * 3;
	
	if ((fread(&planes, 2, 1, in)) != 1) 
	    return;
	if (planes != 1) 
	    return;
	
	if((i = fread(&bpp, 2, 1, in)) != 1) 
	    return;
	if (bpp != 24) 
	    return;
	
	// seek past the rest of the bitmap header
	fseek(in, 24, SEEK_CUR);
	
	// Read the data
	m_image = (unsigned char *) malloc(size);
	
	if((i = fread(m_image, size, 1, in)) != 1) 
	    return;
	
	// reverse all of the colours bgr => rgb)
	for(i=0; i < size; i +=3) 
	{
	    temp           = m_image[i];
	    m_image[i]     = m_image[i + 2];
	    m_image[i + 2] = temp;
	}
	
	m_sizeX = sizeX;
	m_sizeY = sizeY;
    }
    
    /** loadTexture
     *     loads a png file into an opengl texture object, using cstdio , libpng, and opengl.
     * 
     *     \param filename : the png file to be loaded
     *     \param width : width of png, to be updated as a side effect of this function
     *     \param height : height of png, to be updated as a side effect of this function
     * 
     *     \return GLuint : an opengl texture id.  Will be 0 if there is a major error,
     *                                     should be validated by the client of this function.
     * 
     */
    void GTexture::ReadPNG(FILE *in) 
    {
	int width, height, ignore;
	
	//header for testing if it is a png
	png_byte header[8];
	
	//read the header
	ignore = fread(header, 1, 8, in);
	 
	//test if png
	int is_png = !png_sig_cmp(header, 0, 8);
	if (!is_png) 
	{
	    printf("error: not png\n");	    
	    return;
	}
	
	
	//create png struct
	png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png_ptr)
	{
	    printf("error png_ptr\n");	    
	    return;
	}
	
	
	//create png info struct
	png_infop info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr) 
	{
	    png_destroy_read_struct(&png_ptr, (png_infopp) NULL, (png_infopp) NULL);
	    printf("error info_ptr\n");	    
	    return;
	}
	
	//create png info struct
	png_infop end_info = png_create_info_struct(png_ptr);
	if (!end_info) 
	{
	    png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) NULL);
	    printf("error end_info\n");	    
	    return;	    
	}

	//png error stuff, not sure libpng man suggests this.
	if (setjmp(png_jmpbuf(png_ptr))) 
	{
	    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
	    printf("error setjump\n");	    
	    return;	    
	}
	
	//init png reading
	png_init_io(png_ptr, in);
	
	//let libpng know you already read the first 8 bytes
	png_set_sig_bytes(png_ptr, 8);
	
	// read all the info up to the image data
	png_read_info(png_ptr, info_ptr);
	
	//variables to pass to get info
	int bit_depth, color_type;
	png_uint_32 twidth, theight;
	
	// get info about png
	png_get_IHDR(png_ptr, info_ptr, &twidth, &theight, &bit_depth, &color_type,
		     NULL, NULL, NULL);
	
	//update width and height based on png info
	m_sizeX = width = twidth;
	m_sizeY = height= theight;
	
	// Update the png info struct.
	png_read_update_info(png_ptr, info_ptr);
	
	// Row size in bytes.
	int rowbytes = png_get_rowbytes(png_ptr, info_ptr);
	
	// Allocate the image_data as a big block, to be given to opengl
	png_byte *image_data = new png_byte[rowbytes * height];
	if (!image_data) 
	{
	    //clean up memory and close stuff
	    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
	    printf("error image_data\n");	    
	    return;
	}
	
	//row_pointers is for pointing to image_data for reading the png with libpng
	png_bytep *row_pointers = new png_bytep[height];
	if (!row_pointers) 
	{
	    //clean up memory and close stuff
	    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
	    delete[] image_data;
	    printf("error row_pointers\n");	    
	    return;
	}
	// set the individual row_pointers to point at the correct offsets of image_data
	for (int i = 0; i < height; ++i)
	    row_pointers[height - 1 - i] = image_data + i * rowbytes;
	
	//read the png into image_data through row_pointers
	png_read_image(png_ptr, row_pointers);
	
	m_image = (unsigned char*)malloc(sizeof(unsigned char) * rowbytes * height);
	memcpy(m_image, image_data, sizeof(unsigned char) * rowbytes * height);

	//clean up memory and close stuff
	png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
	delete[] image_data;
	delete[] row_pointers;

    }

    void GTexture::SetFileName(const char fname[])
    {
	if(m_fname != fname)
	{
	    if(m_fname)
		free(m_fname);
	    m_fname = strdup(fname);
	}
    }
    
    void GTexture::ManualCoords(void)
    {
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); //GL_DECAL);//GL_MODULATE);
	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);	
    }
    
    void GTexture::AutomaticCoords(void)
    {
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); //GL_DECAL);//GL_MODULATE);	
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	GLfloat s_params[4] = {1.0f, 1.0f, 0.0f, 0.0f};
	GLfloat t_params[4] = {0.0f, 1.0f, 1.0f, 0.0f};
	glTexGenfv(GL_S, GL_OBJECT_PLANE, s_params);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, t_params);

  }
    

    void GTexture::Use(void)
    {
	if(m_fname == NULL || m_image != NULL)
	{
	    glBindTexture(GL_TEXTURE_2D, m_id);
	    return;
	}
	
	m_id    = 0;
	
	const int length = strlen(m_fname);
	if(length >= 4)
	{
	    if(StrSameContent(&m_fname[length - 4], ".ppm"))
	    {
		FILE *in = fopen(m_fname, "rb");
		if(in)
		{
		    m_rgba = false;		    
		    ReadPPM(in);
		    fclose(in);
		}
	    }
	    else if(StrSameContent(&m_fname[length - 4], ".png"))
	    {
		FILE *in = fopen(m_fname, "rb");
		if(in)
		{
		    m_rgba = true;		    
		    ReadPNG(in);
		    fclose(in);
		}
	    }
	    else if(StrSameContent(&m_fname[length - 4], ".bmp"))
	    {
		FILE *in = fopen(m_fname, "r");		    
		if(in)
		{
		    m_rgba = false;		    
		    ReadBMP(in);
		    fclose(in);
		}
	    }
	}
	
	if(m_image)
	{
	    if(m_rgba == false)
	    {
		m_rgba = true;
		ToRGBA();
	    }
	    
	    glGenTextures(1, &m_id);
	    glBindTexture(GL_TEXTURE_2D, m_id);

	    
	    if(m_rgba)
		gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, m_sizeX, m_sizeY, GL_RGBA, GL_UNSIGNED_BYTE, m_image);
	    else
		gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, m_sizeX, m_sizeY, GL_RGB, GL_UNSIGNED_BYTE, m_image);
	    
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    
	}
	
    }

    void GTexture::ToRGBA(void)
    {
	unsigned char *rgba = (unsigned char*) malloc(4 * m_sizeX * m_sizeY);
	
	for(int i = 0; i < m_sizeX * m_sizeY; ++i)
	{
	    rgba[4 * i] = m_image[3 * i];
	    rgba[4 * i + 1] = m_image[3 * i + 1];
	    rgba[4 * i + 2] = m_image[3 * i + 2];
	    rgba[4 * i + 3] = 255;
	}
	free(m_image);
	m_image = rgba;
	
    }
    
}

