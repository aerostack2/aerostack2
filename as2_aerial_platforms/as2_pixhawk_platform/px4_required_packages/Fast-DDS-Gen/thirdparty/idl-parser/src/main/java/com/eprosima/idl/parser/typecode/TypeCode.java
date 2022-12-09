// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package com.eprosima.idl.parser.typecode;

import com.eprosima.idl.parser.tree.Annotation;
import com.eprosima.idl.parser.tree.Notebook;
import com.eprosima.idl.context.Context;

import java.util.Map;
import java.util.HashMap;
import java.util.Collection;
import org.antlr.stringtemplate.StringTemplate;
import org.antlr.stringtemplate.StringTemplateGroup;



public abstract class TypeCode implements Notebook
{
    public static StringTemplateGroup idltypesgr  = null;
    public static StringTemplateGroup cpptypesgr  = null;
    public static StringTemplateGroup ctypesgr    = null;
    public static StringTemplateGroup javatypesgr = null;
    public static Context ctx = null;
    //TODO Revisar si es el mejor sitio.
    public static String javapackage = "";

    public TypeCode(int kind)
    {
        m_kind = kind;
        m_annotations = new HashMap<String, Annotation>();
    }

    public int getKind()
    {
        return m_kind;
    }

    public boolean isIsAnyTypeCode()
    {
        return m_kind == Kind.KIND_NULL;
    }

    /*|
     * @brief This function returns the typename with the scope that is obtained using the cpptypesgr string template.
     * @return The IDL typename.
     */
    public abstract String getCppTypename();

    public abstract String getCTypename();

    protected StringTemplate getCppTypenameFromStringTemplate()
    {
        return cpptypesgr.getInstanceOf("type_" + Integer.toHexString(m_kind));
    }

    protected StringTemplate getCTypenameFromStringTemplate()
    {
        return ctypesgr.getInstanceOf("type_" + Integer.toHexString(m_kind));
    }

    public abstract String getJavaTypename();

    protected StringTemplate getJavaTypenameFromStringTemplate()
    {
        StringTemplate st = javatypesgr.getInstanceOf("type_" + Integer.toHexString(m_kind));
        st.setAttribute("package", javapackage);
        return st;
    }

    /*|
     * @brief This function returns a typename with scope that is obtained using the m_stringtemplatetypesgr string template.
     * @return The typename.
     */
    public abstract String getIdlTypename();

    protected StringTemplate getIdlTypenameFromStringTemplate()
    {
        return idltypesgr.getInstanceOf("type_" + Integer.toHexString(m_kind));
    }

    /*!
     * @brief This function returns the type as a string: "type_2", where the number is the type kind.
     * This function is used in stringtemplates.
     */
    public String getStType()
    {
        return "type_" + Integer.toHexString(m_kind);
    }

    // By default a typecode is not primitive. Function used in stringtemplates
    // TODO Cambiar a isIsPrimitive
    public boolean isPrimitive()
    {
        return false;
    }

    // By default there is not initial value. Function used in stringtemplates.
    public String getInitialValue()
    {
        return "";
    }

    public String getJavaInitialValue()
    {
        return getInitialValue();
    }

    protected String getInitialValueFromStringTemplate()
    {
        Map initialValues = cpptypesgr.getMap("initialValues");
        return initialValues.get(getStType()).toString();
    }

    // By default a typecode doesn't have a max size limit. Function used in stringtemplates
    public String getMaxsize()
    {
        return null;
    }

    /*!
     * @brief This function returns the size of the datatype. By default is null string.
     * @return The size of the datatype.
     */
    protected String getSize()
    {
        return null;
    }

    //public abstract Pair<Integer, Integer> getMaxSerializedSize(int currentSize, int lastDataAligned);

    //public abstract int getMaxSerializedSizeWithoutAlignment(int currentSize);

    /*** Functions to know the type in string templates ***/
    // By default a typecode is not string. Function used in stringtemplates
    public boolean isIsType_5(){return false;}
    public boolean isIsType_6(){return false;}
    public boolean isIsType_7(){return false;}
    public boolean isIsType_d(){return false;}
    public boolean isIsType_c(){return false;}
    public boolean isIsType_f(){return false;}
    public boolean isIsType_e(){return false;}
    public boolean isIsType_a(){return false;}
    public boolean isIsType_10(){return false;}
    public boolean isIsType_13(){return false;}
    public boolean isIsBitmaskType(){return false;}
    public boolean isIsBitsetType(){return false;}
    public boolean isIsStringType() { return false;}
    public boolean isIsWStringType() { return false;}
    public boolean isIsWCharType() { return false;}
    public boolean isIsSetType() { return false; }
    public boolean isIsMapType() { return false; }
    public boolean isIsSequenceType() { return false; }
    public boolean isIsArrayType() { return false; }
    public boolean isIsStructType() {return isIsType_a(); }
    public boolean isIsUnionType() {return m_kind == Kind.KIND_UNION; }

    // Functions to ease TypeIdentifier and TypeObject generation.
    public String getTypeIdentifier() { return "TK_None"; }
    public boolean isPrimitiveType() { return false; }
    public boolean isPlainType() { return false; }
    public boolean isObjectType() { return false; }

    /*** End of functions to know the type in string templates ***/

    public Object getParent()
    {
        return m_parent;
    }

    public void setParent(Object parent)
    {
        m_parent = parent;
    }

    @Override
    public void addAnnotation(Context ctx, Annotation annotation)
    {
        if(annotation != null)
            m_annotations.put(annotation.getName(), annotation);
    }

    @Override
    public Map<String, Annotation> getAnnotations()
    {
        return m_annotations;
    }

    public Collection<Annotation> getAnnotationList()
    {
        return m_annotations.values();
    }

    public boolean isAnnotationFinal()
    {
        Annotation ann = m_annotations.get("final");
        if (ann != null)
        {
            return true;
        }
        ann = m_annotations.get("extensibility");
        if (ann != null)
        {
            return ann.getValue().equals("FINAL");
        }
        return false;
    }

    public boolean isAnnotationAppendable()
    {
        Annotation ann = m_annotations.get("appendable");
        if (ann != null)
        {
            return true;
        }
        ann = m_annotations.get("extensibility");
        if (ann != null)
        {
            return ann.getValue().equals("APPENDABLE");
        }
        return false;
    }

    public boolean isAnnotationMutable()
    {
        Annotation ann = m_annotations.get("mutable");
        if (ann != null)
        {
            return true;
        }
        ann = m_annotations.get("extensibility");
        if (ann != null)
        {
            return ann.getValue().equals("MUTABLE");
        }
        return false;
    }

    public boolean isAnnotationNested()
    {
        Annotation ann = m_annotations.get("nested");
        if (ann != null)
        {
            return ann.getValue().toUpperCase().equals("TRUE");
        }
        return false;
    }

    public boolean isForwarded()
    {
        return m_forwarded;
    }

    public void setForwarded(boolean fwd)
    {
        m_forwarded = fwd;
    }

    public boolean isDefined()
    {
        return m_defined;
    }

    public void setDefined()
    {
        m_defined = true;
    }

    private int m_kind = Kind.KIND_NULL;

    // Added parent object to typecode because was needed in DDS with our types (TopicsPlugin_gettypecode)
    private Object m_parent = null;

    private HashMap<String, Annotation> m_annotations = null;

    private boolean m_forwarded = false;

    private boolean m_defined = false;
}
