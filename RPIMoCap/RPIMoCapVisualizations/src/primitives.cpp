#include <RPIMoCap/primitives.h>


namespace RPIMoCap::Visualization {

Camera::Camera(Eigen::Affine3f transform, Qt3DCore::QEntity *rootEntity) {
    setTransform(transform);

    cameraMesh->setRings(4);
    cameraMesh->setSlices(4);
    cameraMesh->setLength(10);
    cameraMesh->setTopRadius(6.0f);
    cameraMesh->setBottomRadius(0.1f);
    cameraMesh->setHasBottomEndcap(true);
    cameraMesh->setHasTopEndcap(true);

    cameraMaterial->setDiffuse(QColor(QRgb(0x575757)));

    entity->addComponent(cameraTransform);
    entity->addComponent(cameraMesh);
    entity->addComponent(cameraMaterial);
    entity->setParent(rootEntity);
}

void Camera::setTransform(Eigen::Affine3f transform)
{
    Eigen::Affine3f coneTransform = Eigen::Affine3f(Eigen::Affine3f::Identity());
    coneTransform.rotate(Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX()));
    coneTransform.rotate(Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitY()));
    transform = transform * coneTransform;

    cameraTransform->setTranslation({transform.translation().x(),
                                     transform.translation().y(),
                                     transform.translation().z()});

    const Eigen::Quaternionf rot(transform.rotation());
    cameraTransform->setRotation(QQuaternion(QVector4D(rot.x(),rot.y(),rot.z(),rot.w())));
}

Marker::Marker(const Frame::Marker marker, Qt3DCore::QEntity *rootEntity) {
    sphereTransform->setTranslation({marker.position.x(),marker.position.y(),marker.position.z()});

    sphereMesh->setRings(20);
    sphereMesh->setSlices(20);
    sphereMesh->setRadius(2);

    sphereMaterial->setDiffuse(QColor(QRgb(0x0026ff)));

    entity->addComponent(sphereTransform);
    entity->addComponent(sphereMesh);
    entity->addComponent(sphereMaterial);
    entity->setParent(rootEntity);
}

void Marker::setMarker(const Frame::Marker marker)
{
    sphereTransform->setTranslation({marker.position.x(),marker.position.y(),marker.position.z()});
}

Line::Line(const Line3D &line, float lengthcm, Qt3DCore::QEntity *rootEntity)
{
    buf = new Qt3DRender::QBuffer(geometry);
    setLine3D(line, lengthcm);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(2);
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry

    // connectivity between vertices
    QByteArray indexBytes;
    indexBytes.resize(2 * sizeof(unsigned int)); // start to end
    unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
    *indices++ = 0;
    *indices++ = 1;

    auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(2);
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometryS

    // mesh
    renderer->setGeometry(geometry);
    renderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    lineMaterial->setAmbient(QColor(QRgb(0x26e61c)));

    entity->addComponent(renderer);
    entity->addComponent(lineMaterial);
    entity->setParent(rootEntity);
}

void Line::setLine3D(const Line3D &line, const float lengthcm)
{
    bufferBytes.resize(3 * 2 * sizeof(float)); // start.x, start.y, start.end + end.x, end.y, end.z
    float *positions = reinterpret_cast<float*>(bufferBytes.data());
    *positions++ = line.origin().x();
    *positions++ = line.origin().y();
    *positions++ = line.origin().z();

    const auto end = line.origin() + lengthcm * line.direction();
    *positions++ = end.x();
    *positions++ = end.y();
    *positions++ = end.z();

    buf->setData(bufferBytes);
}

FloorPlane::FloorPlane(Qt3DCore::QEntity *rootEntity) {
    entity->addComponent(transform);
    entity->addComponent(mesh);
    entity->addComponent(material);
    entity->setParent(rootEntity);

    material->setDiffuse(QColor(QRgb(0xe3e3e3)));
    material->setShininess(0.0f);

    setSize(500.0f);
    setFloorPosition(-10.0f);
}

void FloorPlane::setSize(const float sizeCM) {
    mesh->setHeight(sizeCM);
    mesh->setWidth(sizeCM);
}

void FloorPlane::setFloorPosition(const float posY) {
    transform->setTranslation({0.0, 0.0, posY});
}

}
