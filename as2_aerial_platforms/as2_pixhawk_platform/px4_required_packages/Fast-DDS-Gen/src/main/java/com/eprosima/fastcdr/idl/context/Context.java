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

package com.eprosima.fastcdr.idl.context;

public interface Context
{
    /*** Functions to select what to print in string templates ***/

    /*!
     * @brief This function is used to stringtemplates to know if the exception type
     * has to be printed.
     */
    public boolean isPrintexception();

    /*!
     * @brief This function is used to stringtemplates to know if the operation type
     * has to be printed.
     */
    public boolean isPrintoperation();

    /*!
     * @brief This function is used to stringtemplates to set the includes location.
     */
    public String getProduct();

    public String getNamespace();

    public String getHeaderGuardName ();

    public boolean isGenerateTypeObject();

    public boolean isGenerateTypesC();

    public boolean isGenerateTypesROS2();

    public boolean isCdr();

    public boolean isFastcdr();

    /*** End ***/
}
