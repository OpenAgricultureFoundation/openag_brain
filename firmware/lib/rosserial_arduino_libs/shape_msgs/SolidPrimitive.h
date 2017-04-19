#ifndef _ROS_shape_msgs_SolidPrimitive_h
#define _ROS_shape_msgs_SolidPrimitive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace shape_msgs
{

  class SolidPrimitive : public ros::Msg
  {
    public:
      uint8_t type;
      uint8_t dimensions_length;
      float st_dimensions;
      float * dimensions;
      enum { BOX = 1 };
      enum { SPHERE = 2 };
      enum { CYLINDER = 3 };
      enum { CONE = 4 };
      enum { BOX_X = 0 };
      enum { BOX_Y = 1 };
      enum { BOX_Z = 2 };
      enum { SPHERE_RADIUS = 0 };
      enum { CYLINDER_HEIGHT = 0 };
      enum { CYLINDER_RADIUS = 1 };
      enum { CONE_HEIGHT = 0 };
      enum { CONE_RADIUS = 1 };

    SolidPrimitive():
      type(0),
      dimensions_length(0), dimensions(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset++) = dimensions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < dimensions_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dimensions[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint8_t dimensions_lengthT = *(inbuffer + offset++);
      if(dimensions_lengthT > dimensions_length)
        this->dimensions = (float*)realloc(this->dimensions, dimensions_lengthT * sizeof(float));
      offset += 3;
      dimensions_length = dimensions_lengthT;
      for( uint8_t i = 0; i < dimensions_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_dimensions));
        memcpy( &(this->dimensions[i]), &(this->st_dimensions), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "shape_msgs/SolidPrimitive"; };
    const char * getMD5(){ return "d8f8cbc74c5ff283fca29569ccefb45d"; };

  };

}
#endif