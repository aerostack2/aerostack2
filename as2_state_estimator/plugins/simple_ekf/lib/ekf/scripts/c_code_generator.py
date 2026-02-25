#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Casadi EKF C Code Generator."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import casadi as ca
import sys
import os
import shutil
from pathlib import Path
project_src = Path(__file__).resolve().parents[1]
print(f'Adding {project_src} to sys.path')
sys.path.insert(0, str(project_src))
from ekf_definition.ekf import EKF


def main():
    """
    Main function to generate C code for the EKF.
    """
    # Create an instance of the EKF
    ekf = EKF()

    # Get functions
    predict_function = ekf.predict_function
    update_pose_function = ekf.update_pose_function
    update_velocity_function = ekf.update_velocity_function

    # Define the options for the code generation
    opts = {
        'with_header': True,
        'verbose': True,
        'cpp': True,
    }

    # Generate C code
    c = ca.CodeGenerator('ekf_c_code', opts)
    c.add(predict_function)
    c.add(update_pose_function)
    c.add(update_velocity_function)
    c.generate()

    # Check if previous files exist, ask for confirmation to overwrite
    if os.path.exists('../src/ekf_c_code.cpp'):
        confirm = input(
            'Previous C code found. Do you want to overwrite it? (y/N): ')
        if confirm.lower() != 'y':
            print('C code generation aborted.')
            return
        else:
            print('Overwriting previous C code...')
            try:
                os.remove('../src/ekf_c_code.cpp')
                os.remove('../include/ekf/ekf_c_code.h')
            except OSError as e:
                print(f'Error removing old files: {e}')
                return

    # Move the generated files to the appropriate directories
    try:
        shutil.copy('ekf_c_code.cpp', '../src/ekf_c_code.cpp')
        shutil.copy('ekf_c_code.h', '../include/ekf/ekf_c_code.h')
        print('C code generated successfully.')
    except OSError as e:
        print(f'Error copying generated files: {e}')

    # Clean up the generated files
    try:
        os.remove('ekf_c_code.cpp')
        os.remove('ekf_c_code.h')
    except OSError as e:
        print(f'Error removing temporary files: {e}')

    print('C code generation completed.')


if __name__ == '__main__':
    main()
