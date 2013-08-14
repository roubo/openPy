#!/usr/bin/python2.7
# vim: set fileencoding=utf-8
"""
	使用Google Calendar API V2
	实现读写自己日历的功能
	roubo_dai@san412.in
	roubo
	在代码风格上会有意模仿谷歌工程师
"""
__author__ = 'roubo_dai@san412.in'

# 根据不同的python版本进行导入“元素树”库
try:
	from xml.etree import ElementTree
except ImportError:
	from elementtree import ElementTree
# 导入必要系统库
import getopt 
import sys
import string 
import time 
# 导入特有google data库
import gdata.calendar.data
import gdata.calendar.client
import gdata.acl.data
import atom 

# 所有日历功能以类封装
class RouboCalendar:
	def __init__(self, email, passsword):
		"""
			google服务认证，即登陆。
			使用google现有的三种认证方式的一种Clientlogin方法。
		"""
		self.cal_client = gdata.calendar.client.CalendarClient(source = 'RouboSyS-Google_Calendar_python-1.0')
		self.cal_client.ClientLogin(email, passsword, self.cal_client.source)

	def _PrintUserCalendars(self):
		"""
			输出用户的所有日历项目
			原理上通过访问https://www.google.com/calendar/feeds/default/allcalendars/full获得列表。
			gdata函数解析了该文本。
		"""
		feed = self.cal_client.GetAllCalendarsFeed()
    		print 'Printing allcalendars: %s' % feed.title.text
    		for i, a_calendar in zip(xrange(len(feed.entry)), feed.entry):
      			print '\t%s. %s' % (i, a_calendar.title.text,)

  	def _PrintOwnCalendars(self):
    		"""
			输出用户owner的日历项目
			原理上是访问https://www.google.com/calendar/feeds/default/owncalendars/full获得列表。
		"""
		feed = self.cal_client.GetOwnCalendarsFeed()
    		print 'Printing owncalendars: %s' % feed.title.text
    		for i, a_calendar in zip(xrange(len(feed.entry)), feed.entry):
     			 print '\t%s. %s' % (i, a_calendar.title.text,)

	def _PrintAllEventsOnDefaultCalendar(self):
    		"""
			输出默认日历的所有事件。
		"""
    		feed = self.cal_client.GetCalendarEventFeed()
    		print 'Events on Primary Calendar: %s' % (feed.title.text,)
    		for i, an_event in zip(xrange(len(feed.entry)), feed.entry):
      			print '\t%s. %s' % (i, an_event.title.text,)
      			for p, a_participant in zip(xrange(len(an_event.who)), an_event.who):
        			print '\t\t%s. %s' % (p, a_participant.email,)
        			print '\t\t\t%s' % (a_participant.value,)
        			if a_participant.attendee_status:
          				print '\t\t\t%s' % (a_participant.attendee_status.value,)

	def _DateRangeQuery(self, start_date='2013-08-14', end_date='2013-08-15'):
    		"""
			按日期索引输出事件
		"""
    		print 'Date range query for events on Primary Calendar: %s to %s' % (start_date, end_date,)
    		query = gdata.calendar.client.CalendarEventQuery(start_min=start_date, start_max=end_date)
    		feed = self.cal_client.GetCalendarEventFeed(q=query)
    		for i, an_event in zip(xrange(len(feed.entry)), feed.entry):
      			print '\t%s. %s' % (i, an_event.title.text,)
      			for a_when in an_event.when:
        			print '\t\tStart time: %s' % (a_when.start,)
        			print '\t\tEnd time:   %s' % (a_when.end,)

	def _InsertEvent(self, title='Tennis with Beth',
	    content='Meet for a quick lesson', where='On the courts',
            start_time=None, end_time=None, recurrence_data=None):
    	"""
		插入事件。
	"""
    	event = gdata.calendar.data.CalendarEventEntry()
    	event.title = atom.data.Title(text=title)
    	event.content = atom.data.Content(text=content)
    	event.where.append(gdata.data.Where(value=where))
	if recurrence_data is not None:
      		# Set a recurring event
      		event.recurrence = gdata.data.Recurrence(text=recurrence_data)
    	else:
      		if start_time is None:
        	# Use current time for the start_time and have the event last 1 hour
        		start_time = time.strftime('%Y-%m-%dT%H:%M:%S.000Z', time.gmtime())
        		end_time = time.strftime('%Y-%m-%dT%H:%M:%S.000Z',time.gmtime(time.time() + 3600))
      		event.when.append(gdata.data.When(start=start_time,end=end_time))
    	new_event = self.cal_client.InsertEvent(event)

    	return new_event

	def _InsertSingleEvent(self, title='One-time Tennis with Beth',
      		content='Meet for a quick lesson', where='On the courts',
      		start_time=None, end_time=None):
    		"""
			插入单一事件。
		"""
    		new_event = self._InsertEvent(title, content, where, start_time, end_time,recurrence_data=None)
		print 'New single event inserted: %s' % (new_event.id.text,)
    		print '\tEvent edit URL: %s' % (new_event.GetEditLink().href,)
    		print '\tEvent HTML URL: %s' % (new_event.GetHtmlLink().href,)
		return new_event

  	def _InsertRecurringEvent(self, title='Weekly Tennis with Beth',
      		content='Meet for a quick lesson', where='On the courts',
      		recurrence_data=None):
    		"""
        		插入循环事件。
		"""
    		if recurrence_data is None:
      			recurrence_data = ('DTSTART;VALUE=DATE:20070501\r\n'
        			+ 'DTEND;VALUE=DATE:20070502\r\n'
        			+ 'RRULE:FREQ=WEEKLY;BYDAY=Tu;UNTIL=20070904\r\n')

    		new_event = self._InsertEvent(title, content, where,
        		recurrence_data=recurrence_data, start_time=None, end_time=None)
		print 'New recurring event inserted: %s' % (new_event.id.text,)
    		print '\tEvent edit URL: %s' % (new_event.GetEditLink().href,)
    		print '\tEvent HTML URL: %s' % (new_event.GetHtmlLink().href,)
		return new_event

  	def _InsertQuickAddEvent(self,
      		content="Tennis with John today 3pm-3:30pm"):
    		"""
			插入快速事件。
		"""
    		event = gdata.calendar.data.CalendarEventEntry()
    		event.content = atom.data.Content(text=content)
    		event.quick_add = gdata.calendar.data.QuickAddProperty(value='true')
    		new_event = self.cal_client.InsertEvent(event)
    		return new_event
	
	def Run(self):
		"""
			运行内容。
		"""
		#self._PrintUserCalendars()
		#self._PrintOwnCalendars()
		self._PrintAllEventsOnDefaultCalendar()	
		self._DateRangeQuery()
def main():
	"""
		主函数。
		解析用户选项，调用run。
	"""
	try:
		opts, args = getopt.getopt(sys.argv[1:], "", ["user=", "pw="])
	except getopt.error, msg:
		print ('python GoogleCal.py --user [username] --pw [password]')
		sys.exit(2)

	user = ''
	pw = ''
	for o, a in opts :
		if o == "--user":
			user = a
		elif o == "--pw":
			pw = a
	if user == '' or pw == '':
		print ('python GoogleCal.py --user [username] --pw [password]')
		sys.exit(2)
	cal = RouboCalendar(user, pw)
	cal.Run()

if __name__ == '__main__':
	main() 
