
import hebi
from time import sleep

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.group import Group


def try_get_group_until_found(lookup: hebi.Lookup, family: str, names: 'list[str]', message: str, delay: float=1.0) -> 'Group':
    group = lookup.get_group_from_names(family, names)
    while group is None:
        print(message)
        sleep(delay)
        group = lookup.get_group_from_names(family, names)
    return group