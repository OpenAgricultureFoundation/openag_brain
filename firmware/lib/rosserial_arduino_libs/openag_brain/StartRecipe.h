#ifndef _ROS_SERVICE_StartRecipe_h
#define _ROS_SERVICE_StartRecipe_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace openag_brain
{

static const char STARTRECIPE[] = "openag_brain/StartRecipe";

  class StartRecipeRequest : public ros::Msg
  {
    public:
      const char* recipe_id;

    StartRecipeRequest():
      recipe_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_recipe_id = strlen(this->recipe_id);
      memcpy(outbuffer + offset, &length_recipe_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->recipe_id, length_recipe_id);
      offset += length_recipe_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_recipe_id;
      memcpy(&length_recipe_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_recipe_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_recipe_id-1]=0;
      this->recipe_id = (char *)(inbuffer + offset-1);
      offset += length_recipe_id;
     return offset;
    }

    const char * getType(){ return STARTRECIPE; };
    const char * getMD5(){ return "42778cde9c99e65254dd2a20a8353e64"; };

  };

  class StartRecipeResponse : public ros::Msg
  {
    public:
      bool success;
      const char* message;

    StartRecipeResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      memcpy(outbuffer + offset, &length_message, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      memcpy(&length_message, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    const char * getType(){ return STARTRECIPE; };
    const char * getMD5(){ return "937c9679a518e3a18d831e57125ea522"; };

  };

  class StartRecipe {
    public:
    typedef StartRecipeRequest Request;
    typedef StartRecipeResponse Response;
  };

}
#endif
