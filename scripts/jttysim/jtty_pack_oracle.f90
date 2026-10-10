! Oracle driver for JTTY's text packer, pack_jtty (WSJT-X 3.2.0-rc1 / 3.3.0-beta1).
!
! Not upstream code. It links upstream's jtty_mod and prints, for each line
! read from standard input, what pack_jtty makes of it, so that a Rust port can
! be tested at exactly the packer's boundary: the same text and exchange
! profile in, the same frames out.
!
! Input, one case per line:  <profile digit>[n] <TAB> <message text>
!   profile 0 = unknown, 1 = Field Day, 2 = RTTY Roundup (jtty_mod's values).
!   A trailing `n` passes is_final=0 (beta1: no end-of-message flag on the last frame); without
!   it is_final is absent, which means final.
!   The text is taken as it stands (up to 80 characters, as pack_jtty's
!   character*80 argument); it may have leading, trailing and repeated spaces.
! Output, one line per case:  <profile>[n] <TAB> <message> <TAB> <nframes> <TAB> <frames>
!   <TAB> <frame_starts>
!   nframes is pack_jtty's (-1 = rejected, 0 = empty message); frames are the
!   34-bit payloads as nine hex digits each (the bits as an integer, first bit
!   most significant), separated by spaces. frame_starts (beta1) are pack_jtty's 1-indexed start
!   columns of each frame's source text in the normalised message, separated by spaces; empty
!   when there are no frames. Needs WSJT-X v3.3.0-beta1 or later: rc1's pack_jtty has neither
!   the frame_starts nor the is_final argument.
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
  integer :: profile, nframes, ios, i, j, k, p, starts(MAX_FRAMES), ifin, mstart
  logical :: partial
  character(len=200) :: outstarts
  character(len=8) :: num
  integer(kind=8) :: v

  do
     read(*,'(a)',iostat=ios) line
     if (ios /= 0) exit
     if (len_trim(line) < 2) cycle
     read(line(1:1),'(i1)') profile
     partial = line(2:2) == 'n'
     mstart = 3
     if (partial) mstart = 4
     message = line(mstart:)
     starts = 0
     ifin = 0
     ! what the caller passed, for the report (pack_jtty rewrites `message`)
     if (partial) then
        call pack_jtty(message, c32, nframes, profile, frame_starts=starts, is_final=ifin)
     else
        call pack_jtty(message, c32, nframes, profile, frame_starts=starts)
     end if
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
     outstarts = ''
     p = 1
     do i = 1, max(nframes, 0)
        write(num,'(i0)') starts(i)
        outstarts(p:p+len_trim(num)-1) = trim(num)
        p = p + len_trim(num) + 1
     end do
     if (partial) then
        write(*,'(i1,a,a,a,a,i0,a,a,a,a)') profile, 'n', achar(9), trim(line(mstart:)), achar(9), nframes, &
             achar(9), trim(outframes), achar(9), trim(outstarts)
     else
        write(*,'(i1,a,a,a,i0,a,a,a,a)') profile, achar(9), trim(line(mstart:)), achar(9), nframes, &
             achar(9), trim(outframes), achar(9), trim(outstarts)
     end if
  end do
end program jtty_pack_oracle
