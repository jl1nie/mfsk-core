// Oracle driver for PSK Reporter's IPFIX packets (WSJT-X Network/PSKReporterIPFIX.cpp).
//
// Not upstream code. It links upstream's PSKReporterIPFIX.cpp (compiled from an exported tag by
// scripts/pskreporter/build_ipfix_oracle.sh, Qt5Core only) and prints, for a script of receivers,
// spots and build parameters, the packets buildPackets makes, so a Rust port can be tested
// byte for byte.
//
// Input, one command per line, fields separated by TAB:
//   P <udp|tcp> <include_descriptors 0|1> <sequence> <observation_id> <export_time>
//   R <callsign> <locator> <program_info> <antenna> <rig_information>
//   S <callsign> <locator> <snr> <frequency_hz> <mode> <unix_time>
//   GO                      build with what was given, print, and clear the spots
// Output, per packet:  K <spot_count> <payload as lower-case hex>
//
// A string may be written as {rep:N:TEXT}: TEXT repeated N times (the long-field cases).
#include <cstdio>
#include <iostream>
#include <string>
#include <QByteArray>
#include <QDateTime>
#include <QList>
#include <QString>
#include "Network/PSKReporterIPFIX.hpp"

static QString expand (std::string const& s)
{
  if (s.rfind ("{rep:", 0) == 0 && s.back () == '}')
    {
      auto const a = s.find (':', 5);
      int const n = std::stoi (s.substr (5, a - 5));
      QString const text = QString::fromUtf8 (s.substr (a + 1, s.size () - a - 2).c_str ());
      QString out;
      for (int i = 0; i < n; ++i) out += text;
      return out;
    }
  return QString::fromUtf8 (s.c_str ());
}

static std::vector<std::string> split (std::string const& line)
{
  std::vector<std::string> f;
  std::string cur;
  for (char c : line)
    {
      if (c == '\t') { f.push_back (cur); cur.clear (); }
      else cur += c;
    }
  f.push_back (cur);
  return f;
}

int main ()
{
  PSKReporterIPFIX::Receiver rx;
  QList<PSKReporterIPFIX::Spot> spots;
  bool tcp = false, desc = false;
  quint32 seq = 0, obs = 0, exp = 0;
  std::string line;
  while (std::getline (std::cin, line))
    {
      if (line.empty () || line[0] == '#') continue;
      auto const f = split (line);
      if (f[0] == "P")
        {
          tcp = f[1] == "tcp"; desc = f[2] == "1";
          seq = std::stoul (f[3]); obs = std::stoul (f[4]); exp = std::stoul (f[5]);
        }
      else if (f[0] == "R")
        {
          rx = {expand (f[1]), expand (f[2]), expand (f[3]), expand (f[4]), expand (f[5])};
        }
      else if (f[0] == "S")
        {
          spots.append ({expand (f[1]), expand (f[2]), std::stoi (f[3]), std::stoull (f[4]),
                         expand (f[5]), QDateTime::fromSecsSinceEpoch (std::stoll (f[6]), Qt::UTC)});
        }
      else if (f[0] == "GO")
        {
          auto const packets = PSKReporterIPFIX::buildPackets (
            rx, spots, desc, seq, obs, exp,
            tcp ? PSKReporterIPFIX::maxTcpIpfixPayloadBytes () : PSKReporterIPFIX::maxUdpIpfixPayloadBytes ());
          for (auto const& p : packets)
            std::printf ("K %d %s\n", p.spot_count, p.payload.toHex ().constData ());
          std::printf ("E %d %d\n", (int) packets.size (), tcp ? 1 : 0);
          spots.clear ();
        }
    }
  return 0;
}
