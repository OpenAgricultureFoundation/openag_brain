#ifndef _ROS_openag_brain_DiagnosticArray_h
#define _ROS_openag_brain_DiagnosticArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "openag_brain/DiagnosticStatus.h"

namespace openag_brain
{

  class DiagnosticArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t status_length;
      openag_brain::DiagnosticStatus st_status;
      openag_brain::DiagnosticStatus * status;

    DiagnosticArray():
      header(),
      status_length(0), status(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = status_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t status_lengthT = *(inbuffer + offset++);
      if(status_lengthT > status_length)
        this->status = (openag_brain::DiagnosticStatus*)realloc(this->status, status_lengthT * sizeof(openag_brain::DiagnosticStatus));
      offset += 3;
      status_length = status_lengthT;
      for( uint8_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(openag_brain::DiagnosticStatus));
      }
     return offset;
    }

    const char * getType(){ return "openag_brain/DiagnosticArray"; };
    const char * getMD5(){ return "f8ed33cf10984329a1cf10f77b980639"; };

  };

}
#endif