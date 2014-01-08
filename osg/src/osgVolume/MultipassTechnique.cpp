/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2009 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include <osgVolume/MultipassTechnique>
#include <osgVolume/VolumeTile>
#include <osgVolume/VolumeScene>

#include <osg/Geometry>
#include <osg/ValueObject>
#include <osg/io_utils>

#include <osg/Program>
#include <osg/Material>
#include <osg/CullFace>
#include <osg/TexGen>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TexEnvFilter>
#include <osg/TransferFunction>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

namespace osgVolume
{

osg::Image* createDownsampledImage(osg::Image* sourceImage)
{
    osg::ref_ptr<osg::Image> targetImage = new osg::Image;

    int s_base = sourceImage->s()/2;
    int s_odd = (sourceImage->s()%2);
    int s = s_base + s_odd;

    int t_base = sourceImage->t()/2;
    int t_odd = (sourceImage->t()%2);
    int t = t_base + t_odd;

    int r_base = sourceImage->r()/2;
    int r_odd = (sourceImage->r()%2);
    int r = r_base + r_odd;

    OSG_NOTICE<<"createDownsampledImage("<<sourceImage->s()<<", "<<sourceImage->t()<<", "<<sourceImage->r()<<")"<<std::endl;
    OSG_NOTICE<<"  s_base = "<<s_base<<", s_odd = "<<s_odd<<", s="<<s<<std::endl;
    OSG_NOTICE<<"  t_base = "<<t_base<<", t_odd = "<<t_odd<<", t="<<t<<std::endl;
    OSG_NOTICE<<"  r_base = "<<r_base<<", r_odd = "<<r_odd<<", r="<<r<<std::endl;

    targetImage->allocateImage(s, t, r, sourceImage->getPixelFormat(), sourceImage->getDataType());

    int numComponents = 1;

    for(int plane=0; plane<r_base; ++plane)
    {
        for(int row=0; row<t_base; ++row)
        {
#if 0
            typedef unsigned char T;
            typedef unsigned short TL;
#else
            typedef unsigned short T;
            typedef unsigned int TL;
#endif
            T* ptr_source_1 = reinterpret_cast<T*>(sourceImage->data(0,row*2, plane*2));
            T* ptr_source_2 = reinterpret_cast<T*>(sourceImage->data(1,row*2, plane*2));
            T* ptr_source_3 = reinterpret_cast<T*>(sourceImage->data(0,row*2+1, plane*2));
            T* ptr_source_4 = reinterpret_cast<T*>(sourceImage->data(1,row*2+1, plane*2));
            T* ptr_source_5 = reinterpret_cast<T*>(sourceImage->data(0,row*2, plane*2+1));
            T* ptr_source_6 = reinterpret_cast<T*>(sourceImage->data(1,row*2, plane*2+1));
            T* ptr_source_7 = reinterpret_cast<T*>(sourceImage->data(0,row*2+1, plane*2+1));
            T* ptr_source_8 = reinterpret_cast<T*>(sourceImage->data(1,row*2+1, plane*2+1));
            T* ptr_target = reinterpret_cast<T*>(targetImage->data(0,row,plane));

            for(int column=0; column<s_base; ++column)
            {
                // average and copy across the source data.
                for(int i=0; i<numComponents; ++i)
                {
#if 1
                    TL value = static_cast<TL>(*ptr_source_1)+
                            static_cast<TL>(*ptr_source_2)+
                            static_cast<TL>(*ptr_source_3)+
                            static_cast<TL>(*ptr_source_4)+
                            static_cast<TL>(*ptr_source_5)+
                            static_cast<TL>(*ptr_source_6)+
                            static_cast<TL>(*ptr_source_7)+
                            static_cast<TL>(*ptr_source_8);
                    value /= 8;
#else
                    TL value = 0;
                    value = osg::maximum(static_cast<TL>(*ptr_source_1),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_2),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_3),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_4),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_5),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_6),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_7),value);
                    value = osg::maximum(static_cast<TL>(*ptr_source_8),value);
#endif
                    *ptr_target = static_cast<T>(value);

                    ++ptr_target;

                    ++ptr_source_1;
                    ++ptr_source_2;
                    ++ptr_source_3;
                    ++ptr_source_4;
                    ++ptr_source_5;
                    ++ptr_source_6;
                    ++ptr_source_7;
                    ++ptr_source_8;
                }
                // skip to the next set of source texels
                ptr_source_1 += numComponents;
                ptr_source_2 += numComponents;
                ptr_source_3 += numComponents;
                ptr_source_4 += numComponents;
                ptr_source_5 += numComponents;
                ptr_source_6 += numComponents;
                ptr_source_7 += numComponents;
                ptr_source_8 += numComponents;
            }
            if (s_odd)
            {

            }
        }
        if (t_odd)
        {
            OSG_NOTICE<<"Need to handle odd image row"<<std::endl;
        }
    }
    if (r_odd)
    {
        OSG_NOTICE<<"Need to handle odd image plane"<<std::endl;
    }

    return targetImage.release();
}


MultipassTechnique::MultipassTechnique()
{
}

MultipassTechnique::MultipassTechnique(const MultipassTechnique& fft,const osg::CopyOp& copyop):
    VolumeTechnique(fft,copyop)
{
}

MultipassTechnique::~MultipassTechnique()
{
}

osg::StateSet* MultipassTechnique::createStateSet(osg::StateSet* statesetPrototype, osg::Program* programPrototype, osg::Shader* shaderToAdd1, osg::Shader* shaderToAdd2)
{
    osg::ref_ptr<osg::StateSet> stateset = osg::clone(statesetPrototype, osg::CopyOp::SHALLOW_COPY);
    osg::ref_ptr<osg::Program> program = osg::clone(programPrototype, osg::CopyOp::SHALLOW_COPY);
    stateset->setAttribute(program.get());
    if (shaderToAdd1) program->addShader(shaderToAdd1);
    if (shaderToAdd2) program->addShader(shaderToAdd2);

    return stateset.release();
}

void MultipassTechnique::init()
{
    OSG_INFO<<"MultipassTechnique::init()"<<std::endl;

    if (!_volumeTile)
    {
        OSG_NOTICE<<"MultipassTechnique::init(), error no volume tile assigned."<<std::endl;
        return;
    }

    if (_volumeTile->getLayer()==0)
    {
        OSG_NOTICE<<"MultipassTechnique::init(), error no layer assigend to volume tile."<<std::endl;
        return;
    }

    if (_volumeTile->getLayer()->getImage()==0)
    {
        OSG_NOTICE<<"MultipassTechnique::init(), error no image assigned to layer."<<std::endl;
        return;
    }

    OSG_NOTICE<<"MultipassTechnique::init() Need to set up"<<std::endl;

    CollectPropertiesVisitor cpv(false);
    if (_volumeTile->getLayer()->getProperty())
    {
        _volumeTile->getLayer()->getProperty()->accept(cpv);
    }

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    {
        osg::Geometry* geom = new osg::Geometry;

        osg::Vec3Array* coords = new osg::Vec3Array(8);
        (*coords)[0] = osg::Vec3d(0.0,0.0,0.0);
        (*coords)[1] = osg::Vec3d(1.0,0.0,0.0);
        (*coords)[2] = osg::Vec3d(1.0,1.0,0.0);
        (*coords)[3] = osg::Vec3d(0.0,1.0,0.0);
        (*coords)[4] = osg::Vec3d(0.0,0.0,1.0);
        (*coords)[5] = osg::Vec3d(1.0,0.0,1.0);
        (*coords)[6] = osg::Vec3d(1.0,1.0,1.0);
        (*coords)[7] = osg::Vec3d(0.0,1.0,1.0);
        geom->setVertexArray(coords);

        osg::Vec4Array* colours = new osg::Vec4Array(1);
        (*colours)[0].set(1.0f,1.0f,1.0,1.0f);
        geom->setColorArray(colours, osg::Array::BIND_OVERALL);

        osg::DrawElementsUShort* drawElements = new osg::DrawElementsUShort(GL_QUADS);
        // bottom
        drawElements->push_back(3);
        drawElements->push_back(2);
        drawElements->push_back(1);
        drawElements->push_back(0);

        // bottom
        drawElements->push_back(7);//7623
        drawElements->push_back(6);
        drawElements->push_back(2);
        drawElements->push_back(3);

        // left
        drawElements->push_back(4);//4730
        drawElements->push_back(7);
        drawElements->push_back(3);
        drawElements->push_back(0);

        // right
        drawElements->push_back(1);//1265
        drawElements->push_back(2);
        drawElements->push_back(6);
        drawElements->push_back(5);

        // front
        drawElements->push_back(5);//5401
        drawElements->push_back(4);
        drawElements->push_back(0);
        drawElements->push_back(1);

        // top
        drawElements->push_back(4);//4567
        drawElements->push_back(5);
        drawElements->push_back(6);
        drawElements->push_back(7);

        geom->addPrimitiveSet(drawElements);

        geode->addDrawable(geom);

    }

    _transform = new osg::MatrixTransform;
    _transform->addChild(geode.get());

    // handle locators
    Locator* masterLocator = _volumeTile->getLocator();
    Locator* layerLocator = _volumeTile->getLayer()->getLocator();

    osg::TransferFunction1D* tf = 0;

    if (!masterLocator && layerLocator) masterLocator = layerLocator;
    if (!layerLocator && masterLocator) layerLocator = masterLocator;

    osg::Matrix geometryMatrix;
    if (masterLocator)
    {
        geometryMatrix = masterLocator->getTransform();
        _transform->setMatrix(geometryMatrix);
        masterLocator->addCallback(new TransformLocatorCallback(_transform.get()));
    }

    osg::Matrix imageMatrix;
    if (layerLocator)
    {
        imageMatrix = layerLocator->getTransform();
    }

    OSG_NOTICE<<"MultipassTechnique::init() : geometryMatrix = "<<geometryMatrix<<std::endl;
    OSG_NOTICE<<"MultipassTechnique::init() : imageMatrix = "<<imageMatrix<<std::endl;

    osg::ref_ptr<osg::StateSet> stateset = _transform->getOrCreateStateSet();

    unsigned int texgenTextureUnit = 0;
    unsigned int volumeTextureUnit = 2;

    // set up uniforms
    {
        stateset->addUniform(new osg::Uniform("colorTexture",0));
        stateset->addUniform(new osg::Uniform("depthTexture",1));

        stateset->setMode(GL_ALPHA_TEST,osg::StateAttribute::ON);

        float alphaFuncValue = 0.1;
        if (cpv._isoProperty.valid())
        {
            alphaFuncValue = cpv._isoProperty->getValue();
        }

        if (cpv._sampleRatioProperty.valid())
            stateset->addUniform(cpv._sampleRatioProperty->getUniform());
        else
            stateset->addUniform(new osg::Uniform("SampleRatioValue",1.0f));


        if (cpv._transparencyProperty.valid())
            stateset->addUniform(cpv._transparencyProperty->getUniform());
        else
            stateset->addUniform(new osg::Uniform("TransparencyValue",1.0f));


        if (cpv._afProperty.valid())
            stateset->addUniform(cpv._afProperty->getUniform());
        else
            stateset->addUniform(new osg::Uniform("AlphaFuncValue",alphaFuncValue));


        if (cpv._isoProperty.valid())
            stateset->addUniform(cpv._isoProperty->getUniform());
        else
            stateset->addUniform(new osg::Uniform("IsoSurfaceValue",alphaFuncValue));

        if (cpv._tfProperty.valid())
        {
            tf = dynamic_cast<osg::TransferFunction1D*>(cpv._tfProperty->getTransferFunction());
        }

        osg::ref_ptr<osg::TexGen> texgen = new osg::TexGen;
        texgen->setMode(osg::TexGen::OBJECT_LINEAR);
        texgen->setPlanesFromMatrix( geometryMatrix * osg::Matrix::inverse(imageMatrix));

        if (masterLocator)
        {
            osg::ref_ptr<TexGenLocatorCallback> locatorCallback = new TexGenLocatorCallback(texgen, masterLocator, layerLocator);
            masterLocator->addCallback(locatorCallback.get());
            if (masterLocator != layerLocator)
            {
                if (layerLocator) layerLocator->addCallback(locatorCallback.get());
            }
        }

        stateset->setTextureAttributeAndModes(texgenTextureUnit, texgen.get(), osg::StateAttribute::ON);
    }


    // set up 3D texture
    osg::ref_ptr<osg::Image> image_3d = _volumeTile->getLayer()->getImage();

    // create a downsampled image to use when rendering at a lower quality.
    // osg::ref_ptr<osg::Image> downsampled_image_3d = createDownsampledImage(image_3d.get());

    //image_3d = createDownsampledImage(downsampled_image_3d.get());
    // image_3d = downsampled_image_3d;

    osg::ref_ptr<osg::Texture3D> texture3D = new osg::Texture3D;
    {
        osg::Texture::InternalFormatMode internalFormatMode = osg::Texture::USE_IMAGE_DATA_FORMAT;
#define VOLUME_TYPE 2
#if VOLUME_TYPE==1
        osg::Texture::FilterMode minFilter = osg::Texture::LINEAR_MIPMAP_LINEAR;
        osg::Texture::FilterMode magFilter = osg::Texture::LINEAR;
#elif VOLUME_TYPE==2
        osg::Texture::FilterMode minFilter = osg::Texture::LINEAR;
        osg::Texture::FilterMode magFilter = osg::Texture::LINEAR;
#else
        osg::Texture::FilterMode minFilter = osg::Texture::NEAREST;
        osg::Texture::FilterMode magFilter = osg::Texture::NEAREST;
#endif

        // set up the 3d texture itself,
        // note, well set the filtering up so that mip mapping is disabled,
        // gluBuild3DMipsmaps doesn't do a very good job of handled the
        // imbalanced dimensions of the 256x256x4 texture.
        texture3D->setResizeNonPowerOfTwoHint(false);
        texture3D->setFilter(osg::Texture3D::MIN_FILTER,minFilter);
        texture3D->setFilter(osg::Texture3D::MAG_FILTER, magFilter);
        texture3D->setWrap(osg::Texture3D::WRAP_R,osg::Texture3D::CLAMP_TO_BORDER);
        texture3D->setWrap(osg::Texture3D::WRAP_S,osg::Texture3D::CLAMP_TO_BORDER);
        texture3D->setWrap(osg::Texture3D::WRAP_T,osg::Texture3D::CLAMP_TO_BORDER);
        //texture3D->setMaxAnisotropy(16.0f);
        texture3D->setBorderColor(osg::Vec4(0.0,0.0,0.0,0.0));
        if (image_3d->getPixelFormat()==GL_ALPHA ||
            image_3d->getPixelFormat()==GL_LUMINANCE)
        {
            texture3D->setInternalFormatMode(osg::Texture3D::USE_USER_DEFINED_FORMAT);
            texture3D->setInternalFormat(GL_INTENSITY);
        }
        else
        {
            texture3D->setInternalFormatMode(internalFormatMode);
        }
        texture3D->setImage(image_3d);

        stateset->setTextureAttributeAndModes(volumeTextureUnit, texture3D, osg::StateAttribute::ON);

        osg::ref_ptr<osg::Uniform> baseTextureSampler = new osg::Uniform("volumeTexture", int(volumeTextureUnit));
        stateset->addUniform(baseTextureSampler.get());

        osg::ref_ptr<osg::Uniform> volumeCellSize = new osg::Uniform("volumeCellSize", osg::Vec3(1.0f/static_cast<float>(image_3d->s()),1.0f/static_cast<float>(image_3d->t()),1.0f/static_cast<float>(image_3d->r())));
        stateset->addUniform(volumeCellSize.get());

        OSG_NOTICE<<"Texture Dimensions "<<image_3d->s()<<", "<<image_3d->t()<<", "<<image_3d->r()<<std::endl;
    }

    if (tf)
    {
        OSG_NOTICE<<"Setting up TransferFunction"<<std::endl;

        float tfScale = 1.0f;
        float tfOffset = 0.0f;

        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(_volumeTile->getLayer());
        if (imageLayer)
        {
            tfOffset = (imageLayer->getTexelOffset()[3] - tf->getMinimum()) / (tf->getMaximum() - tf->getMinimum());
            tfScale = imageLayer->getTexelScale()[3] / (tf->getMaximum() - tf->getMinimum());
        }
        else
        {
            tfOffset = -tf->getMinimum() / (tf->getMaximum()-tf->getMinimum());
            tfScale = 1.0f / (tf->getMaximum()-tf->getMinimum());
        }
        osg::ref_ptr<osg::Texture1D> tf_texture = new osg::Texture1D;
        tf_texture->setImage(tf->getImage());

        tf_texture->setResizeNonPowerOfTwoHint(false);
        tf_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        tf_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        tf_texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_EDGE);

        unsigned int transferFunctionTextureUnit = volumeTextureUnit+1;

        stateset->setTextureAttributeAndModes(transferFunctionTextureUnit, tf_texture.get(), osg::StateAttribute::ON);
        stateset->addUniform(new osg::Uniform("tfTexture",int(transferFunctionTextureUnit)));
        stateset->addUniform(new osg::Uniform("tfOffset",tfOffset));
        stateset->addUniform(new osg::Uniform("tfScale",tfScale));

    }

    // creates CullFace attributes to apply to front/back StateSet configurations.
    osg::ref_ptr<osg::CullFace> front_CullFace = new osg::CullFace(osg::CullFace::BACK);
    osg::ref_ptr<osg::CullFace> back_CullFace = new osg::CullFace(osg::CullFace::FRONT);



    osg::ref_ptr<osg::Shader> computeRayColorShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_compute_ray_color.frag");
#if 0
    if (!computeRayColorShader)
    {
        #include "Shaders/volume_compute_ray_color_frag.cpp";
        computeRayColorShader = new osg::Shader(osg::Shader::FRAGMENT, volume_compute_ray_color_frag);
    }
#endif

    osg::ref_ptr<osg::Shader> main_vertexShader = osgDB::readRefShaderFile(osg::Shader::VERTEX, "shaders/volume_multipass.vert");
#if 0
    if (!main_vertexShader)
    {
        #include "Shaders/volume_multipass_vert.cpp"
        main_vertexShader = new osg::Shader(osg::Shader::VERTEX, volume_multipass_vert));
    }
#endif

    osg::ref_ptr<osg::Shader> front_main_fragmentShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_multipass_front.frag");
#if 0
    if (!front_main_fragmentShader)
    {
        #include "Shaders/volume_multipass_front_frag.cpp"
        front_main_fragmentShader = new osg::Shader(osg::Shader::VERTEX, volume_multipass_front_frag));
    }
#endif


    osg::ref_ptr<osg::Shader> back_main_fragmentShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_multipass_back.frag");
#if 0
    if (!back_main_fragmentShader)
    {
        #include "Shaders/volume_multipass_back_frag.cpp"
        back_main_fragmentShader = new osg::Shader(osg::Shader::VERTEX, volume_multipass_back_frag));
    }
#endif

    // clear any previous settings
    _stateSetMap.clear();

    osg::ref_ptr<osg::StateSet> front_stateset_prototype = new osg::StateSet;
    osg::ref_ptr<osg::Program> front_program_prototype = new osg::Program;
    {
        front_stateset_prototype->setAttributeAndModes(front_CullFace.get(), osg::StateAttribute::ON);

        front_program_prototype->addShader(main_vertexShader.get());
        front_program_prototype->addShader(front_main_fragmentShader.get());
        front_program_prototype->addShader(computeRayColorShader.get());
    }

    osg::ref_ptr<osg::StateSet> back_stateset_prototype = new osg::StateSet;
    osg::ref_ptr<osg::Program> back_program_prototype = new osg::Program;
    {
        back_stateset_prototype->setAttributeAndModes(back_CullFace.get(), osg::StateAttribute::ON);

        back_program_prototype->addShader(main_vertexShader.get());
        back_program_prototype->addShader(back_main_fragmentShader.get());
        back_program_prototype->addShader(computeRayColorShader.get());
    }

    // STANDARD_SHADERS
    {
        // STANDARD_SHADERS without TransferFunction
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_standard.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_standard_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_standard_frag);
            }
            #endif

            // front
            _stateSetMap[STANDARD_SHADERS|FRONT_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[STANDARD_SHADERS|BACK_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }

        // STANDARD_SHADERS with TransferFunction
        if (tf)
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_standard_tf.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_standard_tf_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_standard_tf_frag);
            }
            #endif


            // front
            _stateSetMap[STANDARD_SHADERS|FRONT_SHADERS|TF_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[STANDARD_SHADERS|BACK_SHADERS|TF_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }
    }

    // ISO_SHADERS
    if (cpv._isoProperty.valid())
    {
        // ISO_SHADERS without TransferFunction
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_iso.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_iso_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_iso_frag);
            }
            #endif

            // front
            _stateSetMap[ISO_SHADERS|FRONT_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[ISO_SHADERS|BACK_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }

        // ISO_SHADERS with TransferFunction
        if (tf)
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_iso_tf.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_standard_iso_tf_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_standard_iso_tf_frag);
            }
            #endif


            // front
            _stateSetMap[ISO_SHADERS|FRONT_SHADERS|TF_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[ISO_SHADERS|BACK_SHADERS|TF_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }
    }

    // MIP_SHADERS
    if (cpv._mipProperty.valid())
    {
        // MIP_SHADERS without TransferFunction
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_mip.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_mip_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_mip_frag);
            }
            #endif

            // front
            _stateSetMap[MIP_SHADERS|FRONT_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[MIP_SHADERS|BACK_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }

        // MIP_SHADERS with TransferFunction
        if (tf)
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_mip_tf.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_standard_mip_tf_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_standard_mip_tf_frag);
            }
            #endif


            // front
            _stateSetMap[MIP_SHADERS|FRONT_SHADERS|TF_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[MIP_SHADERS|BACK_SHADERS|TF_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }
    }

    // LIT_SHADERS
    if (cpv._lightingProperty.valid())
    {
        // LIT_SHADERS without TransferFunction
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_lit.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_lit_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_lit_frag);
            }
            #endif

            // front
            _stateSetMap[LIT_SHADERS|FRONT_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[LIT_SHADERS|BACK_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }

        // MIP_SHADERS with TransferFunction
        if (tf)
        {
            osg::ref_ptr<osg::Shader> accumulateSamplesShader = osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "shaders/volume_accumulateSamples_lit_tf.frag");
            #if 0
            if (!accumulateSamplesShader)
            {
                #include "Shaders/volume_accumulateSamples_standard_lit_tf_frag.cpp";
                accumulateSamplesShader = new osg::Shader(osg::Shader::FRAGMENT, volume_accumulateSamples_standard_lit_tf_frag);
            }
            #endif


            // front
            _stateSetMap[LIT_SHADERS|FRONT_SHADERS|TF_SHADERS] = createStateSet(front_stateset_prototype.get(), front_program_prototype.get(), accumulateSamplesShader.get());

            // back
            _stateSetMap[LIT_SHADERS|BACK_SHADERS|TF_SHADERS] = createStateSet(back_stateset_prototype.get(), back_program_prototype.get(), accumulateSamplesShader.get());
        }
    }


    if (cpv._sampleRatioWhenMovingProperty.valid())
    {
        _whenMovingStateSet = new osg::StateSet;
        //_whenMovingStateSet->setTextureAttributeAndModes(volumeTextureUnit, new osg::TexEnvFilter(1.0));
        _whenMovingStateSet->addUniform(cpv._sampleRatioWhenMovingProperty->getUniform(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    }

}

void MultipassTechnique::update(osgUtil::UpdateVisitor* /*uv*/)
{
//    OSG_NOTICE<<"MultipassTechnique:update(osgUtil::UpdateVisitor* nv):"<<std::endl;
}

void MultipassTechnique::cull(osgUtil::CullVisitor* cv)
{
    std::string traversalPass;
    bool postTraversal = cv->getUserValue("VolumeSceneTraversal", traversalPass) && traversalPass=="Post";

    // OSG_NOTICE<<"MultipassTechnique::cull()  traversalPass="<<traversalPass<<std::endl;

    if (postTraversal)
    {

        int shaderMask = 0;
        if (_volumeTile->getLayer()->getProperty())
        {
            CollectPropertiesVisitor cpv;
            _volumeTile->getLayer()->getProperty()->accept(cpv);

            if (cpv._tfProperty.valid())
            {
                shaderMask |= TF_SHADERS;
            }

            if (cpv._isoProperty.valid())
            {
                shaderMask |= ISO_SHADERS;
            }
            else if (cpv._mipProperty.valid())
            {
                shaderMask |= MIP_SHADERS;
            }
            else if (cpv._lightingProperty.valid())
            {
                shaderMask |= LIT_SHADERS;
            }
            else
            {
                shaderMask |= STANDARD_SHADERS;
            }
        }

        int shaderMaskFront = shaderMask | FRONT_SHADERS;
        int shaderMaskBack = shaderMask | BACK_SHADERS;

        // OSG_NOTICE<<"shaderMaskFront "<<shaderMaskFront<<std::endl;
        // OSG_NOTICE<<"shaderMaskBack  "<<shaderMaskBack<<std::endl;


        osg::ref_ptr<osg::StateSet> front_stateset = _stateSetMap[shaderMaskFront];
        osg::ref_ptr<osg::StateSet> back_stateset = _stateSetMap[shaderMaskBack];
        osg::ref_ptr<osg::StateSet> moving_stateset = (_whenMovingStateSet.valid() && isMoving(cv)) ? _whenMovingStateSet : 0;

        if (moving_stateset.valid())
        {
            // OSG_NOTICE<<"Using MovingStateSet"<<std::endl;
            cv->pushStateSet(moving_stateset.get());
        }

        if (front_stateset.valid())
        {
            // OSG_NOTICE<<"Have front stateset"<<std::endl;
            cv->pushStateSet(front_stateset.get());
            _transform->accept(*cv);
            cv->popStateSet();
        }

        if (back_stateset.valid())
        {
            // OSG_NOTICE<<"Have back stateset"<<std::endl;
            cv->pushStateSet(back_stateset.get());
            _transform->accept(*cv);
            cv->popStateSet();
        }

        if (moving_stateset.valid())
        {
            // OSG_NOTICE<<"Using MovingStateSet"<<std::endl;
            cv->popStateSet();
        }
    }
    else
    {
        osg::NodePath& nodePath = cv->getNodePath();
        for(osg::NodePath::reverse_iterator itr = nodePath.rbegin();
            itr != nodePath.rend();
            ++itr)
        {
            osgVolume::VolumeScene* vs = dynamic_cast<osgVolume::VolumeScene*>(*itr);
            if (vs)
            {
                vs->tileVisited(cv, getVolumeTile());
                break;
            }
        }
    }
}

void MultipassTechnique::cleanSceneGraph()
{
    OSG_NOTICE<<"MultipassTechnique::cleanSceneGraph()"<<std::endl;
}

void MultipassTechnique::traverse(osg::NodeVisitor& nv)
{
    // OSG_NOTICE<<"MultipassTechnique::traverse(osg::NodeVisitor& nv)"<<std::endl;
    if (!_volumeTile) return;

    // if app traversal update the frame count.
    if (nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_volumeTile->getDirty()) _volumeTile->init();

        osgUtil::UpdateVisitor* uv = dynamic_cast<osgUtil::UpdateVisitor*>(&nv);
        if (uv)
        {
            update(uv);
            return;
        }

    }
    else if (nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            cull(cv);
            return;
        }
    }


    if (_volumeTile->getDirty())
    {
        OSG_INFO<<"******* Doing init ***********"<<std::endl;
        _volumeTile->init();
    }
}


} // end of osgVolume namespace
