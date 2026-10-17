! Oracle for mfsk-core's msg::wsjt77::pack77_message: WSJT-X's own pack77 (the module
! packjt77 of lib/77bit, v3.2.0-rc1), run once per input line from a clean hash state.
!
!   pack77_oracle <cases-file> [prefer]     (any second argument sets prefer_wspr_50bit)
!
! One line in, one tab-separated line out:
!   input  status  i3.n3  bits  decoded  hashed-calls
! status  = ok | not_encoded | free_text_too_long | free_text_invalid |
!           preferred_family_rejected | roundtrip_rejected   (PACK77_STATUS_* of packjt77.f90:66-71)
! decoded = what unpack77 prints for those bits (also from a clean state)
! hashed-calls = the calls pack77 committed to the shared hash table for this message
!           (pack77_hash_facts, oldest first), comma separated; the table is emptied before
!           each message, so what it holds afterwards is exactly this message's facts.
! The input is passed as character(len=80); pack77_encode_result takes its first 37.
program pack77_oracle
  use packjt77
  implicit none
  character(len=80) :: line
  character(len=512) :: infile
  character(len=77) :: c77
  character(len=37) :: msg
  character(len=1000) :: hashed
  character(len=26) :: stext
  integer :: i3, n3, st, nargs, i, nlen
  logical :: ok
  type(pack77_options) :: opt

  nargs = command_argument_count()
  if (nargs < 1) stop 2
  call get_command_argument(1, infile)
  opt%prefer_wspr_50bit = .false.
  if (nargs >= 2) opt%prefer_wspr_50bit = .true.
  open(10, file=trim(infile), status='old')
  do
     read(10, '(a)', end=999) line
     ! clean hash state for every message
     calls10 = ''; calls12 = ''; calls22 = ''; ihash22 = -1; nzhash = 0
     i3 = -1; n3 = -1; c77 = repeat('0', 77)
     call pack77(line, i3, n3, c77, options=opt, status=st)
     select case (st)
     case (PACK77_STATUS_ENCODED);                    stext = 'ok'
     case (PACK77_STATUS_NOT_ENCODED);                stext = 'not_encoded'
     case (PACK77_STATUS_FREE_TEXT_TOO_LONG);         stext = 'free_text_too_long'
     case (PACK77_STATUS_FREE_TEXT_INVALID);          stext = 'free_text_invalid'
     case (PACK77_STATUS_PREFERRED_FAMILY_REJECTED);  stext = 'preferred_family_rejected'
     case (PACK77_STATUS_INTERNAL_ROUNDTRIP_REJECTED); stext = 'roundtrip_rejected'
     case default;                                    stext = 'unknown'
     end select
     hashed = ''
     nlen = 0
     do i = nzhash, 1, -1
        if (nlen > 0) then
           hashed(nlen+1:nlen+1) = ','
           nlen = nlen + 1
        end if
        hashed(nlen+1:nlen+len_trim(calls22(i))) = trim(calls22(i))
        nlen = nlen + len_trim(calls22(i))
     end do
     if (st == PACK77_STATUS_ENCODED) then
        calls10 = ''; calls12 = ''; calls22 = ''; ihash22 = -1; nzhash = 0
        msg = ''
        call unpack77(c77, 0, msg, ok)
        if (.not. ok) msg = '*unpack failed*'
        write(*, '(a,a,a,a,a,i0,a,i0,a,a,a,a,a,a)') trim(line), char(9), trim(stext), char(9), &
             '', i3, '.', n3, char(9), c77, char(9), trim(msg), char(9), hashed(1:nlen)
     else
        write(*, '(a,a,a,a,a,a,a,a,a,a)') trim(line), char(9), trim(stext), char(9), &
             '-', char(9), '-', char(9), '-', char(9)
     end if
  end do
999 continue
end program pack77_oracle
