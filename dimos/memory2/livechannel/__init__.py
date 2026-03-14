# Backwards-compat shim: livechannel/ was renamed to notifier/.
# Kept so that deserialize_component() can resolve old registry entries.
from dimos.memory2.notifier.base import Notifier
from dimos.memory2.notifier.subject import SubjectNotifier

__all__ = ["Notifier", "SubjectNotifier"]
