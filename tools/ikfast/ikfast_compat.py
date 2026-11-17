"""Process-local compatibility fix for OpenRAVE + SymPy 0.7.1."""
from __future__ import print_function
import hashlib
import inspect
import textwrap


def install_and_test(ikfast, sympy):
    cls = ikfast.IKFastSolver
    name = '_SimplifyRotationCross'
    method = getattr(cls, name)
    original = getattr(method, 'im_func', method)
    source = textwrap.dedent(inspect.getsource(original))
    replacements = [
        ('Poly.from_dict({m0:c0},*p.gens)',
         'Poly.from_dict({m0:c0},*p.gens,domain=p.domain)'),
        ('Poly.from_dict({m1:c1},*p.gens)',
         'Poly.from_dict({m1:c1},*p.gens,domain=p.domain)'),
        ('Poly.from_dict({tuple(m2):c0},*p.gens)',
         'Poly.from_dict({tuple(m2):c0},*p.gens,domain=p.domain)'),
    ]
    patched = source
    for old, new in replacements:
        if patched.count(old) != 1:
            raise RuntimeError('Unexpected OpenRAVE source; refusing compatibility patch: ' + old)
        patched = patched.replace(old, new)
    namespace = dict(getattr(original, '__globals__', None)
                     or getattr(original, 'func_globals'))
    eval(compile(patched, '<fr5-ikfast-domain-fix>', 'exec'), namespace)
    replacement = namespace[name]

    # Exercise the real patched method before starting expensive generation.
    # Cross-product identity: a*b-c*d=e. Coefficients contain joint symbols,
    # floats and rationals, as in the reported RR[cj1,cj2,sj1,sj2] failure.
    class EqualOnly(object):
        def equal(self, left, right):
            return sympy.expand(left-right) == 0
    a, b, c, d, e = sympy.symbols('a b c d e')
    cj1, cj2, sj1, sj2 = sympy.symbols('cj1 cj2 sj1 sj2')
    coefficients = [sympy.Integer(2), sympy.Rational(17, 40),
                    sympy.Float('0.39501'), cj1+cj2,
                    sympy.Float('0.39501')*cj1*cj2 + sympy.Float('0.425')*sj1*sj2,
                    sympy.Rational(39501, 100000)*cj1 + sj2]
    groups = [((0, 1), (2, 3), 4)]
    probe = EqualOnly()
    cases = [(coefficient, sympy.Rational(7, 10)) for coefficient in coefficients]
    cases += [(sympy.Float('0.39501'), (cj1*cj2+sj1*sj2)*e**2),
              (sympy.Float('0.425')*cj1, (cj2+sj1+sj2)*e**2)]
    original_failures = 0
    for coefficient, remainder in cases:
        expr = sympy.expand(coefficient*(a*b-c*d) + remainder)
        try:
            original(probe, expr, (a, b, c, d, e), groups)
        except sympy.polys.polyerrors.CoercionFailed:
            original_failures += 1
        result = replacement(probe, expr, (a, b, c, d, e), groups)
        if result is None or sympy.expand(result.as_expr() - coefficient*e - remainder) != 0:
            raise RuntimeError('Cross-product compatibility regression failed')
    if replacement(probe, a+b, (a, b, c, d, e), groups) is not None:
        raise RuntimeError('Compatibility patch changed an unrelated expression')

    setattr(cls, name, replacement)
    return {'fix': 'preserve_polynomial_coefficient_domain', 'regression_cases': len(cases)+1,
            'original_coercion_failures_in_regression': original_failures,
            'original_method_sha256': hashlib.sha256(source.encode('utf-8')).hexdigest(),
            'patched_method_sha256': hashlib.sha256(patched.encode('utf-8')).hexdigest()}
