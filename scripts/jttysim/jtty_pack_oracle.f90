! Oracle driver for JTTY's text packer, pack_jtty (WSJT-X 3.2.0-rc1).
!
! Not upstream code. It links upstream's jtty_mod and prints, for each line
! read from standard input, what pack_jtty makes of it, so that a Rust port can
! be tested at exactly the packer's boundary: the same text and exchange
! profile in, the same frames out.
!
! Input, one case per line:  <profile digit> <TAB> <message text>
!   profile 0 = unknown, 1 = Field Day, 2 = RTTY Roundup (jtty_mod's values).
!   The text is taken as it stands (up to 80 characters, as pack_jtty's
!   character*80 argument); it may have leading, trailing and repeated spaces.
! Output, one line per case:  <profile> <TAB> <message> <TAB> <nframes> <TAB> <frames>
!   nframes is pack_jtty's (-1 = rejected, 0 = empty message); frames are the
!   34-bit payloads as nine hex digits each (the bits as an integer, first bit
!   most significant), separated by spaces.
!
! Usage: jtty_pack_oracle < cases.tsv > pack_cases.tsv

program jtty_pack_oracle
  use jtty_mod, only: pack_jtty, MAX_FRAMES
  implicit none
  character(len=1024) :: line
  character(len=80) :: message
  character(len=34) :: c32(MAX_FRAMES)
  character(len=9) :: hex
  character(len=200) :: outframes
  integer :: profile, nframes, ios, i, j, k, p
  integer(kind=8) :: v

  do
     read(*,'(a)',iostat=ios) line
     if (ios /= 0) exit
     if (len_trim(line) < 2) cycle
     read(line(1:1),'(i1)') profile
     message = line(3:)
     ! what the caller passed, for the report (pack_jtty rewrites `message`)
     call pack_jtty(message, c32, nframes, profile)
     outframes = ''
     p = 1
     do i = 1, max(nframes, 0)
        v = 0
        do j = 1, 34
           v = 2*v
           if (c32(i)(j:j) == '1') v = v + 1
        end do
        write(hex,'(z9.9)') v
        outframes(p:p+8) = hex
        p = p + 10
     end do
     write(*,'(i1,a,a,a,i0,a,a)') profile, achar(9), trim(line(3:)), achar(9), nframes, achar(9), trim(outframes)
  end do
end program jtty_pack_oracle
