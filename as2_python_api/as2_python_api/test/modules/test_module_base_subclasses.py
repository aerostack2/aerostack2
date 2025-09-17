"""Test ModuleBase subclasses."""

# Copyright 2022 Universidad Politécnica de Madrid
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
#    * Neither the name of the the copyright holder nor the names of its
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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import importlib
import inspect
import pkgutil
import unittest

import as2_python_api.modules as modules
from as2_python_api.modules.module_base import ModuleBase


class TestModuleBaseSubclasses(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Dynamically import all modules in the package to ensure subclasses are loaded."""
        package = modules
        for _, module_name, _ in pkgutil.walk_packages(package.__path__, package.__name__ + '.'):
            importlib.import_module(module_name)

    def test_subclasses_implement_call(self):
        """Test that all subclasses of ModuleBase implement the __call__ method."""
        subclasses = ModuleBase.__subclasses__()
        print(subclasses)

        for subclass in subclasses:
            with self.subTest(subclass=subclass):
                # Check if __call__ is implemented in the subclass
                if '__call__' in subclass.__dict__:
                    method = subclass.__dict__['__call__']
                    self.assertFalse(
                        inspect.isabstract(method),
                        f'{subclass.__name__} does not implement the __call__ method.'
                    )
                else:
                    self.fail(f'{subclass.__name__} does not implement the __call__ method.')


if __name__ == '__main__':
    unittest.main()
