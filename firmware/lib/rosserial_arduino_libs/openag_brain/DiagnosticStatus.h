#ifndef _ROS_openag_brain_DiagnosticStatus_h
#define _ROS_openag_brain_DiagnosticStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace openag_brain
{

  class DiagnosticStatus : public ros::Msg
  {
    public:
      int8_t level;
      const char* name;
      uint8_t code;
      enum { OK = 0 };
      enum { WARN = 1 };
      enum { ERROR = 2 };
      enum { STALE = 3 };

    DiagnosticStatus():
      level(0),
      name(""),
      code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_level;
      u_level.real = this->level;
      *(outbuffer + offset + 0) = (u_level.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_level;
      u_level.base = 0;
      u_level.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->level = u_level.real;
      offset += sizeof(this->level);
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->code);
     return offset;
    }

    const char * getType(){ return "openag_brain/DiagnosticStatus"; };
    const char * getMD5(){ return "51483619a2e090b92bc2ed312361df98"; };

  };

}
#endif