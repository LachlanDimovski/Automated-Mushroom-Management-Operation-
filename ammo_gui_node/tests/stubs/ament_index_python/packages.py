import os

ROOT = os.environ.get('AMMO_STUB_SHARE', '')


def get_package_share_directory(pkg):
    """Point at the checkout so layouts/ and worlds/ resolve in tests."""
    if not ROOT:
        raise LookupError(pkg)
    for candidate in (os.path.join(ROOT, pkg), os.path.join(ROOT, pkg, pkg)):
        if os.path.isdir(candidate):
            return candidate
    raise LookupError(pkg)
