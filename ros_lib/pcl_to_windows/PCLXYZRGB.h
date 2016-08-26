#ifndef _ROS_pcl_to_windows_PCLXYZRGB_h
#define _ROS_pcl_to_windows_PCLXYZRGB_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pcl_to_windows
{

  class PCLXYZRGB : public ros::Msg
  {
    public:
      uint32_t cld_total;
      uint32_t cld_num;
      uint32_t x_length;
      float st_x;
      float * x;
      uint32_t y_length;
      float st_y;
      float * y;
      uint32_t z_length;
      float st_z;
      float * z;
      uint32_t r_length;
      float st_r;
      float * r;
      uint32_t g_length;
      float st_g;
      float * g;
      uint32_t b_length;
      float st_b;
      float * b;

    PCLXYZRGB():
      cld_total(0),
      cld_num(0),
      x_length(0), x(NULL),
      y_length(0), y(NULL),
      z_length(0), z(NULL),
      r_length(0), r(NULL),
      g_length(0), g(NULL),
      b_length(0), b(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cld_total >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cld_total >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cld_total >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cld_total >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cld_total);
      *(outbuffer + offset + 0) = (this->cld_num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cld_num >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cld_num >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cld_num >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cld_num);
      *(outbuffer + offset + 0) = (this->x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_length);
      for( uint32_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      *(outbuffer + offset + 0) = (this->y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_length);
      for( uint32_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yi;
      u_yi.real = this->y[i];
      *(outbuffer + offset + 0) = (u_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y[i]);
      }
      *(outbuffer + offset + 0) = (this->z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_length);
      for( uint32_t i = 0; i < z_length; i++){
      union {
        float real;
        uint32_t base;
      } u_zi;
      u_zi.real = this->z[i];
      *(outbuffer + offset + 0) = (u_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z[i]);
      }
      *(outbuffer + offset + 0) = (this->r_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->r_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->r_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->r_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_length);
      for( uint32_t i = 0; i < r_length; i++){
      union {
        float real;
        uint32_t base;
      } u_ri;
      u_ri.real = this->r[i];
      *(outbuffer + offset + 0) = (u_ri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ri.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r[i]);
      }
      *(outbuffer + offset + 0) = (this->g_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->g_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->g_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->g_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->g_length);
      for( uint32_t i = 0; i < g_length; i++){
      union {
        float real;
        uint32_t base;
      } u_gi;
      u_gi.real = this->g[i];
      *(outbuffer + offset + 0) = (u_gi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->g[i]);
      }
      *(outbuffer + offset + 0) = (this->b_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->b_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->b_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->b_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b_length);
      for( uint32_t i = 0; i < b_length; i++){
      union {
        float real;
        uint32_t base;
      } u_bi;
      u_bi.real = this->b[i];
      *(outbuffer + offset + 0) = (u_bi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->cld_total =  ((uint32_t) (*(inbuffer + offset)));
      this->cld_total |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cld_total |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->cld_total |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->cld_total);
      this->cld_num =  ((uint32_t) (*(inbuffer + offset)));
      this->cld_num |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cld_num |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->cld_num |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->cld_num);
      uint32_t x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->x_length);
      if(x_lengthT > x_length)
        this->x = (float*)realloc(this->x, x_lengthT * sizeof(float));
      x_length = x_lengthT;
      for( uint32_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_x;
      u_st_x.base = 0;
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_x = u_st_x.real;
      offset += sizeof(this->st_x);
        memcpy( &(this->x[i]), &(this->st_x), sizeof(float));
      }
      uint32_t y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->y_length);
      if(y_lengthT > y_length)
        this->y = (float*)realloc(this->y, y_lengthT * sizeof(float));
      y_length = y_lengthT;
      for( uint32_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_y;
      u_st_y.base = 0;
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_y = u_st_y.real;
      offset += sizeof(this->st_y);
        memcpy( &(this->y[i]), &(this->st_y), sizeof(float));
      }
      uint32_t z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_length);
      if(z_lengthT > z_length)
        this->z = (float*)realloc(this->z, z_lengthT * sizeof(float));
      z_length = z_lengthT;
      for( uint32_t i = 0; i < z_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_z;
      u_st_z.base = 0;
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_z = u_st_z.real;
      offset += sizeof(this->st_z);
        memcpy( &(this->z[i]), &(this->st_z), sizeof(float));
      }
      uint32_t r_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      r_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      r_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      r_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->r_length);
      if(r_lengthT > r_length)
        this->r = (float*)realloc(this->r, r_lengthT * sizeof(float));
      r_length = r_lengthT;
      for( uint32_t i = 0; i < r_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_r;
      u_st_r.base = 0;
      u_st_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_r = u_st_r.real;
      offset += sizeof(this->st_r);
        memcpy( &(this->r[i]), &(this->st_r), sizeof(float));
      }
      uint32_t g_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      g_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      g_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      g_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->g_length);
      if(g_lengthT > g_length)
        this->g = (float*)realloc(this->g, g_lengthT * sizeof(float));
      g_length = g_lengthT;
      for( uint32_t i = 0; i < g_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_g;
      u_st_g.base = 0;
      u_st_g.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_g.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_g.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_g.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_g = u_st_g.real;
      offset += sizeof(this->st_g);
        memcpy( &(this->g[i]), &(this->st_g), sizeof(float));
      }
      uint32_t b_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      b_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      b_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      b_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->b_length);
      if(b_lengthT > b_length)
        this->b = (float*)realloc(this->b, b_lengthT * sizeof(float));
      b_length = b_lengthT;
      for( uint32_t i = 0; i < b_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_b;
      u_st_b.base = 0;
      u_st_b.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_b.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_b.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_b.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_b = u_st_b.real;
      offset += sizeof(this->st_b);
        memcpy( &(this->b[i]), &(this->st_b), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pcl_to_windows/PCLXYZRGB"; };
    const char * getMD5(){ return "fedc9c50492f0dc997bb690d7724f3b1"; };

  };

}
#endif