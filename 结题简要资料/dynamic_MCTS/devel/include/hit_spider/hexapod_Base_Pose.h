// Generated by gencpp from file hit_spider/hexapod_Base_Pose.msg
// DO NOT EDIT!


#ifndef HIT_SPIDER_MESSAGE_HEXAPOD_BASE_POSE_H
#define HIT_SPIDER_MESSAGE_HEXAPOD_BASE_POSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <hit_spider/hexapod_RPY.h>

namespace hit_spider
{
template <class ContainerAllocator>
struct hexapod_Base_Pose_
{
  typedef hexapod_Base_Pose_<ContainerAllocator> Type;

  hexapod_Base_Pose_()
    : position()
    , orientation()  {
    }
  hexapod_Base_Pose_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , orientation(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::hit_spider::hexapod_RPY_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;





  typedef boost::shared_ptr< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> const> ConstPtr;

}; // struct hexapod_Base_Pose_

typedef ::hit_spider::hexapod_Base_Pose_<std::allocator<void> > hexapod_Base_Pose;

typedef boost::shared_ptr< ::hit_spider::hexapod_Base_Pose > hexapod_Base_PosePtr;
typedef boost::shared_ptr< ::hit_spider::hexapod_Base_Pose const> hexapod_Base_PoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator1> & lhs, const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator2> & rhs)
{
  return lhs.position == rhs.position &&
    lhs.orientation == rhs.orientation;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator1> & lhs, const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hit_spider

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f1b4a886328450f637af10145259080b";
  }

  static const char* value(const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf1b4a886328450f6ULL;
  static const uint64_t static_value2 = 0x37af10145259080bULL;
};

template<class ContainerAllocator>
struct DataType< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hit_spider/hexapod_Base_Pose";
  }

  static const char* value(const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point position\n"
"hit_spider/hexapod_RPY orientation\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: hit_spider/hexapod_RPY\n"
"float64 roll\n"
"float64 pitch\n"
"float64 yaw\n"
;
  }

  static const char* value(const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.orientation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct hexapod_Base_Pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hit_spider::hexapod_Base_Pose_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::hit_spider::hexapod_RPY_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HIT_SPIDER_MESSAGE_HEXAPOD_BASE_POSE_H
