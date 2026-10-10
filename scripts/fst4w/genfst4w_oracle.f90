! Oracle driver for FST4W transmit encoding: WSJT-X's own genfst4 (v3.3.0-beta1).
!
! Not upstream code. For each line of standard input (a message, up to 37
! characters) it calls genfst4(msg, ichk=0, ..., iwspr) with iwspr=1, the way
! fst4sim -W / the FST4W transmitter does, and prints
!
!   <message> TAB <msgsent> TAB <iwspr out> TAB <i3>.<n3> TAB <74 msgbits> TAB <160 tones>
!
! msgsent is "*** bad message ***" for a rejected message (tones all 0, bits 0).
! iwspr out is 1 for every FST4W request (genfst4.f90:62-64, :66-67).
! The 74 message bits are payload (50) + CRC-24 (24). i3.n3 is not exposed by
! genfst4, so it is recomputed with pack77 on the same options.
!
! Output format: message TAB msgsent TAB iwspr TAB i3.n3 TAB bits TAB tones.
! Usage: genfst4w_oracle < messages.txt > tones.tsv
program genfst4w_oracle
  use packjt77
  include 'fst4_params.f90'   ! parameter statements need implicit typing
  character(len=256) :: line
  character(len=37) :: msg, msgsent
  character(len=77) :: c77
  integer :: i4tone(NN), iwspr, ios, i, i3, n3
  integer*1 :: msgbits(101)
  character(len=NN) :: ts
  character(len=74) :: bs
  do
     read(*,'(a)',iostat=ios) line
     if (ios /= 0) exit
     msg = line(1:37)
     iwspr = 1
     i4tone = 0
     msgbits = 0
     call genfst4(msg, 0, msgsent, msgbits, i4tone, iwspr)
     i3 = -1; n3 = -1; c77 = ' '
     call pack77(msg, i3, n3, c77, pack77_options(prefer_wspr_50bit=.true.))
     do i = 1, NN
        ts(i:i) = char(ichar('0') + i4tone(i))
     end do
     do i = 1, 74
        bs(i:i) = char(ichar('0') + msgbits(i))
     end do
     write(*,'(a,a,a,a,i0,a,i0,a,i0,a,a,a,a)') trim(line), char(9), trim(msgsent), char(9), iwspr, &
          char(9), i3, '.', n3, char(9), bs, char(9), ts
  end do
end program
