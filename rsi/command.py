default_command = '''<Sen Type="ImFree">
  <AK A1="{:.4f}" A2="{:.4f}" A3="{:.4f}" A4="{:.4f}" A5="{:.4f}" A6="{:.4f}" />
<IPOC>{IPOC}</IPOC>
</Sen>'''

def make_command(A1, A2, A3, A4, A5, A6, IPOC):
    return default_command.format(A1, A2, A3, A4, A5, A6, IPOC=IPOC)

def get_ipoc(xml):
    start_tag = '<IPOC>'
    end_tag = '</IPOC>'
    sp = xml.find(start_tag) + len(start_tag)
    ep = xml.find(end_tag)
    return xml[sp:ep]

def joint_correction_command(from_kuka, joint_desired):
    IPOC = get_ipoc(from_kuka)
    jd = joint_desired
    cmd = str(make_command(jd[0], jd[1], jd[2], jd[3], jd[4], jd[5], IPOC))
    return cmd

