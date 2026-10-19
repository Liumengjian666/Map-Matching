"""Exact approved-header selection using existing record-time lookup metadata."""
from audit_math import ros_header


def verified_records(bag, topic, header_record, make_time):
    """Check exact indexed records; reject missing/duplicate queried occurrences.

    This does not search other bag record times for additional header copies.
    """
    found = set()
    for record_ns in sorted(set(header_record.values())):
        stamp = make_time(record_ns)
        for actual_topic, raw, record in bag.read_messages(
                topics=[topic], raw=True, start_time=stamp, end_time=stamp):
            header_ns = ros_header(raw[1])['header_ns']
            if header_ns not in header_record:
                continue
            if actual_topic != topic or record.to_nsec() != header_record[header_ns]:
                raise ValueError('RECORD_LOOKUP_MISMATCH')
            if header_ns in found:
                raise ValueError('DUPLICATE_EXPECTED_HEADER')
            found.add(header_ns)
            yield actual_topic, raw, record
    if found != set(header_record):
        raise ValueError('MISSING_EXPECTED_HEADERS')


def indexed_records(rows, topic, wanted):
    """Require one lookup row per selected header before building a mapping."""
    result = {}
    for row in rows:
        header_ns = int(row['stamp_ns'])
        if row['topic'] != topic or not wanted(header_ns):
            continue
        if header_ns in result:
            raise ValueError('DUPLICATE_LOOKUP_HEADER')
        result[header_ns] = int(row['record_ns'])
    return result
