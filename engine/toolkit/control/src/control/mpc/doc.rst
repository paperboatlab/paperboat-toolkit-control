The model is a JSON file on the block:

.. code-block:: json

   { "A": [[1]], "B": [[1]], "Q": [[1]], "R": [[0.1]], "N": 20, "u_min": [-1], "u_max": [1] }

It is condensed into one QP over the horizon ``N`` and solved by the optimization toolkit every step.
