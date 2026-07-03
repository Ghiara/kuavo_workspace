# Changelog

All notable changes to the Kuavo_up will be documented in this file.

## 2026-07-03

### System configuration modification

- **Added Syslog Size Limit**: Updated the `rsyslog` log rotation configuration to prevent `/var/log/syslog` from growing indefinitely when hardware or camera-related errors are repeatedly written.

- **Added Log Retention Limit**: Restricted historical syslog files to a maximum retention period of 2 days to reduce unnecessary system disk usage.

### Configuration modification:

In `/etc/logrotate.d/rsyslog`:

- New lines added:

```conf
maxsize 100M
maxage 2