#include <smarc_gazebo_plugins/SemanticMultiRayShape.hh>

using namespace gazebo;
using namespace physics;

RayShapePtr SemanticMultiRayShape::StaticGetRay(MultiRayShapePtr _parent, unsigned int _index)
{
  return static_cast<SemanticMultiRayShape*>(_parent.get())->GetRay(_index);
}

RayShapePtr SemanticMultiRayShape::GetRay(unsigned int _index)
{
  return rays[_index];
}

SemanticMultiRayShape::SemanticMultiRayShape(CollisionPtr _parent) : MultiRayShape(_parent)
{

}

SemanticMultiRayShape::~SemanticMultiRayShape()
{

}
