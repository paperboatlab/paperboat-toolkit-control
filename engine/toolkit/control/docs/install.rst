Install
=======

The control toolkit links the optimization toolkit, so install both:

.. code-block:: sh

   paperboat toolkit install optimization --repo https://github.com/paperboatlab/paperboat-toolkit-optimization.git
   paperboat toolkit install control --repo https://github.com/paperboatlab/paperboat-toolkit-control.git

A C++ backend registers them in its ``main``:

.. code-block:: cpp

   tk::Registry r = tk::builtins();
   tk::optimization::register_optimization(r);
   tk::control::register_control(r);

and imports ``"optimization"`` and ``"control"`` in its ``descriptor.toml``:

.. code-block:: toml

   import = ["signal", "math", "optimization", "control"]
