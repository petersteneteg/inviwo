#ifndef IVW_CAMERAPROPERTY_H
#define IVW_CAMERAPROPERTY_H

#include "inviwo/core/inviwo.h"
#include "inviwo/core/properties/vectorproperties.h"
#include "inviwo/core/properties/compositeproperty.h"

namespace inviwo {

class CameraProperty : public CompositeProperty {

public:
    CameraProperty(std::string identifier, std::string displayName,
                   vec3 lookFrom=vec3(0.0f, 0.0f, -3.0f), vec3 lookTo=vec3(0.0f), vec3 lookUp=vec3(0.0f, 1.0f, 0.0f));
    virtual ~CameraProperty();

    vec3 lookFrom() const { return lookFrom_.get(); }
    void setLookFrom(vec3 lookFrom);
    vec3 lookTo() const { return lookTo_.get(); }
    void setLookTo(vec3 lookTo);
    vec3 lookUp() const { return lookUp_.get(); }
    void setLookUp(vec3 lookUp);

    mat4 viewMatrix() const { return viewMatrix_; }
    mat4 projectionMatrix() const { return projectionMatrix_; }

    void invalidate();

    virtual void serialize(IvwSerializer& s) const;
    virtual void deserialize(IvwDeserializer& d);

private:
    FloatVec3Property lookFrom_;
    FloatVec3Property lookTo_;
    FloatVec3Property lookUp_;

    mat4 viewMatrix_;
    mat4 projectionMatrix_;

    void updateViewMatrix();
};

} // namespace

#endif // IVW_CAMERAPROPERTY_H
