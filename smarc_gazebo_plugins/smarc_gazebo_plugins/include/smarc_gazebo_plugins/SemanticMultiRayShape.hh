#ifndef SEMANTIC_MULTIRAY_SHAPE_HH
#define SEMANTIC_MULTIRAY_SHAPE_HH

#include <gazebo/physics/MultiRayShape.hh>

namespace gazebo
{

  //class MultiRayShape;

  namespace physics {
    
    /// \class MultiRayShape MultiRayShape.hh physics/physics.hh
    /// \brief Laser collision contains a set of ray-collisions,
    /// structured to simulate a laser range scanner.
    class SemanticMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision shape.
      public: explicit SemanticMultiRayShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~SemanticMultiRayShape();

      public: RayShapePtr GetRay(unsigned int _index);
      
	  public: static RayShapePtr StaticGetRay(MultiRayShapePtr _parent, unsigned int _index);

    };

    typedef boost::shared_ptr<SemanticMultiRayShape> SemanticMultiRayShapePtr;

  }

}

#endif // SEMANTIC_MULTIRAY_SHAPE_HH
