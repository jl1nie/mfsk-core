// Oracle for "is this decode a PSK Reporter spot, and of whom": WSJT-X v3.3.0-beta1's own rules.
//
// Not upstream code. It takes `tokens_re` from Decoder/decodedtext.cpp (cut out of the exported
// file by build_spot_oracle.sh, not retyped), reproduces DecodedText's message_ preparation and
// `deCallAndGrid`, calls the real Fortran `stdmsg_` / `stdmsg72_` from libwsjt_fort for "is a
// standard message" (`stdMsg` in MainWindow::readFromStdout), and applies MainWindow::pskPost's
// test (`grid_regexp`, or a CQ in the line).
//
// Input, one per line, TAB-separated:  <mode>  <message>      (mode: FT8 FT4 FST4 JT9 JT65 Q65 ...)
// Output:  <standard 0|1>  <call>  <grid>  <post 0|1>
//
// Mode `TOKEN` takes one word and answers  <Radio::is_standard_callsign 0|1>  <decoded_grid_pattern 0|1>
// (the JTTY spot rule, JttyReceiveResultController, is built from those two).
//
// `post` is the part of pskPost that depends on the message alone: not the time, the disk-data
// flag, low confidence or self-spotting, which the caller decides.
#include <iostream>
#include <string>
#include <QRegularExpression>
#include <QString>

extern "C" {
  bool stdmsg_ (char const * msg, int len);
  bool stdmsg72_ (char const * msg, bool * is_jt65, int len);
}

#include "tokens_re.inc"

// Radio::is_standard_callsign, cut out of Radio.cpp by build_spot_oracle.sh
#include "std_call.inc"

static QRegularExpression const angle_bracket_re {"[<>]"};
static QRegularExpression const cq_qrz_re {"^(CQ|QRZ)\\s"};
// Radio::decoded_grid_pattern()
static QRegularExpression const grid_regexp {"\\A(?![Rr]{2}73)[A-Ra-r]{2}[0-9]{2}([A-Xa-x]{2}){0,1}\\z"};

int main ()
{
  std::string line;
  while (std::getline (std::cin, line))
    {
      auto const tab = line.find ('\t');
      if (tab == std::string::npos) continue;
      QString const mode = QString::fromUtf8 (line.substr (0, tab).c_str ());
      if (mode == "TOKEN")
        {
          QString const w = QString::fromUtf8 (line.substr (tab + 1).c_str ());
          std::printf ("%d\t%d\n", is_standard_callsign (w) ? 1 : 0, w.contains (grid_regexp) ? 1 : 0);
          continue;
        }
      QString message_ = QString::fromUtf8 (line.substr (tab + 1).c_str ()).trimmed ();
      // DecodedText::DecodedText
      auto const message0_ = message_.left (37);
      message_ = message0_;
      message_.remove (angle_bracket_re);
      if (message_.contains (cq_qrz_re))
        {
          auto eom_pos = message_.indexOf (' ', 16);
          if (eom_pos < 16) eom_pos = message_.size () - 1;
          message_ = message_.left (eom_pos + 1);
        }
      auto c_string = message0_.toLocal8Bit ();
      c_string += QByteArray {37 - c_string.size (), ' '};
      bool standard;
      if (mode == "JT9" || mode == "JT65")
        {
          bool is_jt65 = mode == "JT65";
          standard = stdmsg72_ (c_string.constData (), &is_jt65, 37);
        }
      else
        {
          standard = stdmsg_ (c_string.constData (), 37);
        }
      // DecodedText::deCallAndGrid
      auto msg = message_;
      auto p = msg.indexOf ("; ");
      if (p >= 0) msg = msg.mid (p + 2);
      auto const match = tokens_re.match (msg);
      QString call = match.captured ("word2");
      QString grid = match.captured ("word3");
      if ("R" == grid) grid = match.captured ("word4");
      // MainWindow::pskPost; the line is "HHMMSS  snr  dt freq ~  message", so a CQ message has " CQ "
      QString const string_ = QString ("123456  -10  0.2 1500 ~  ") + message_;
      bool const post = standard && (grid.contains (grid_regexp) || string_.contains (" CQ "));
      std::printf ("%d\t%s\t%s\t%d\n", standard ? 1 : 0, call.toUtf8 ().constData (),
                   grid.toUtf8 ().constData (), post ? 1 : 0);
    }
  return 0;
}
